#!/usr/bin/env python3

import os

import numpy as np
import rclpy
import yaml
from aruco_opencv_msgs.msg import ArucoDetection
from geometry_msgs.msg import Pose, PoseStamped
from rclpy.node import Node
from scipy.spatial.transform import Rotation as R
from sensor_msgs.msg import Image, JointState
from std_msgs.msg import ColorRGBA, Empty, Header, Int32
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker

from agx_arm_vision.moveit2_local import MoveIt2


def _default_config_path():
    here = os.path.dirname(os.path.abspath(__file__))
    candidates = [
        os.path.join(here, '..', '..', 'config', 'shelf_layers.yaml'),
    ]
    try:
        from ament_index_python.packages import get_package_share_directory
        candidates.insert(0, os.path.join(
            get_package_share_directory('agx_arm_vision'),
            'config', 'shelf_layers.yaml'))
    except Exception:
        pass
    for path in candidates:
        if os.path.exists(path):
            return path
    return candidates[-1]


class ShelfWorkflowNode(Node):
    IDLE = 0
    NOMINAL_POSE = 1
    ALIGN = 2
    WAIT_DETECT = 3
    TRIGGER_GRASP = 4
    GRASPING = 5
    WAIT_RELEASE_CMD = 6
    PLACING = 7
    RETURN_HOME = 8

    STATE_NAMES = {
        0: 'IDLE', 1: 'NOMINAL_POSE', 2: 'ALIGN',
        3: 'WAIT_DETECT', 4: 'TRIGGER_GRASP', 5: 'GRASPING',
        6: 'WAIT_RELEASE_CMD', 7: 'PLACING', 8: 'RETURN_HOME',
    }

    EXECUTOR_STATE_NAMES = {
        0: 'IDLE', 1: 'OPEN_GRIPPER', 2: 'MOVE_TO_TARGET',
        3: 'WAIT_REACH', 4: 'CLOSE_GRIPPER', 5: 'MOVE_HOME',
        6: 'WAIT_RELEASE', 7: 'MOVE_TO_PLACE_ABOVE',
        8: 'LOWER_TO_PLACE', 9: 'PLACE_OPEN', 10: 'PLACE_LIFT',
    }

    EXECUTOR_IDLE = 0
    EXECUTOR_WAIT_RELEASE = 6

    ALIGN_DIST_EPS = 0.02#for test remember to change it back to 0.02!!!!在rviz测试时临时使用！！！

    # 精对准目标：aruco 距相机 align_dist 米、其中心投影到画面中心即成功。
    ALIGN_DIST = 0.4          # aruco 距相机目标距离 (m)
    ALIGN_DIST_TOL = 0.05     # 3D 距离容差 (m)
    ALIGN_CENTER_TOL = 0.05   # 归一化画面中心容差 (x/z, y/z)

    # TCP-referenced viewing pose offsets: the TCP lands TCP_BACK_OFFSET
    # toward the arm and TCP_HEIGHT_OFFSET above the marker.
    TCP_BACK_OFFSET = 0.1
    TCP_HEIGHT_OFFSET = 0.2
    # Fixed home tcp_link orientation fallback (xyzw), used when the TF
    # lookup at startup is unavailable.
    HOME_TCP_QUAT = (0.683, 0.692, -0.171, -0.161)

    def __init__(self):
        super().__init__('shelf_workflow_node')
        self.declare_parameter('config_file', _default_config_path())
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('end_effector_link', 'tcp_link')
        self.declare_parameter('camera_frame', 'camera_color_optical_frame')
        self.declare_parameter('arm_group', 'arm')
        self.declare_parameter('velocity_scaling', 0.1)
        self.declare_parameter('aruco_timeout', 5.0)
        self.declare_parameter('align_give_up_timeout', 15.0)
        self.declare_parameter('grasp_fail_timeout', 20.0)
        self.declare_parameter('grasp_give_up_timeout', 90.0)
        self.declare_parameter('place_give_up_timeout', 90.0)
        self.declare_parameter('detect_timeout', 5.0)
        self.declare_parameter('detect_give_up_timeout', 15.0)
        self.declare_parameter('grasp_max_retries', 3)
        self.declare_parameter('target_z_min', 0.01)
        self.declare_parameter('target_z_max', 0.35)
        self.declare_parameter('ret_home_timeout', 90.0)
        self.declare_parameter('ret_home_exec_wait', 60.0)
        self.declare_parameter('align_settle_time', 3.0)
        self.declare_parameter('box_stable_tol', 0.05)
        self.declare_parameter('align_max_iter', 2)
        self.declare_parameter('skip_nominal', True)
        self.declare_parameter('align_dist', 0.4)
        self.declare_parameter('align_dist_tol', 0.05)
        self.declare_parameter('align_center_tol', 0.05)
        self.declare_parameter('settle_after_home', 1.0)
        self.declare_parameter('marker_fresh_timeout', 1.0)
        self.declare_parameter(
            'home_joints',
            [-0.0259, -0.4025, -0.0575, 2.0, 0.0604, 0.0722, 0.9141])

        config_file = self.get_parameter('config_file').value
        if not os.path.exists(config_file):
            self.get_logger().error(
                f'Config file not found: {config_file}')
        with open(config_file, 'r', encoding='utf-8') as f:
            self._cfg = yaml.safe_load(f)
        self.get_logger().info(
            f'Loaded {len(self._cfg.get("layers", []))} shelf layers '
            f'from {config_file}')

        self.base_frame = self.get_parameter('base_frame').value
        self.end_effector = self.get_parameter('end_effector_link').value
        self.camera_frame = self.get_parameter('camera_frame').value
        self.velocity_scaling = self.get_parameter('velocity_scaling').value
        self.aruco_timeout = self.get_parameter('aruco_timeout').value
        self.align_give_up_timeout = self.get_parameter(
            'align_give_up_timeout').value
        self.grasp_fail_timeout = self.get_parameter(
            'grasp_fail_timeout').value
        self.grasp_give_up_timeout = self.get_parameter(
            'grasp_give_up_timeout').value
        self.place_give_up_timeout = self.get_parameter(
            'place_give_up_timeout').value
        self.detect_timeout = self.get_parameter('detect_timeout').value
        self.detect_give_up_timeout = self.get_parameter(
            'detect_give_up_timeout').value
        self.grasp_max_retries = int(self.get_parameter(
            'grasp_max_retries').value)
        self.target_z_min = float(self.get_parameter('target_z_min').value)
        self.target_z_max = float(self.get_parameter('target_z_max').value)
        self.ret_home_timeout = self.get_parameter(
            'ret_home_timeout').value
        self.ret_home_exec_wait = self.get_parameter(
            'ret_home_exec_wait').value
        self.align_settle_time = float(
            self.get_parameter('align_settle_time').value)
        self.box_stable_tol = float(
            self.get_parameter('box_stable_tol').value)
        self.align_max_iter = self.get_parameter('align_max_iter').value
        self.skip_nominal = bool(self.get_parameter('skip_nominal').value)
        self.align_dist = float(self.get_parameter('align_dist').value)
        self.align_dist_tol = float(
            self.get_parameter('align_dist_tol').value)
        self.align_center_tol = float(
            self.get_parameter('align_center_tol').value)
        self.settle_after_home = self.get_parameter(
            'settle_after_home').value
        self.marker_fresh_timeout = self.get_parameter(
            'marker_fresh_timeout').value
        self.home_joints = list(self.get_parameter('home_joints').value)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.arm = MoveIt2(
            node=self,
            base_link=self.base_frame,
            end_effector=self.end_effector,
            group_name=self.get_parameter('arm_group').value,
            constrain_orientation=True,
            position_tolerance=0.01,
            orientation_tolerance=0.15,
        )

        self.grasp_start_pub = self.create_publisher(
            Empty, 'manual_grasp_start', 10)
        self.grasp_cmd_pub = self.create_publisher(
            PoseStamped, 'grasp_target_cmd', 10)
        self.release_pub = self.create_publisher(
            Empty, 'manual_release', 10)
        self.release_force_pub = self.create_publisher(
            Empty, 'manual_release_force', 10)
        self.align_target_pub = self.create_publisher(
            Marker, 'shelf/align_target', 10)

        self.create_subscription(
            Int32, 'task_command', self.task_cmd_cb, 10)
        self.create_subscription(
            Empty, 'release_command', self.release_cmd_cb, 10)
        self.create_subscription(
            ArucoDetection, '/aruco_detections', self.aruco_cb, 10)
        self.create_subscription(
            Marker, 'yolo/target_box', self.target_box_cb, 10)
        self.create_subscription(
            PoseStamped, 'grasp_pose', self.grasp_pose_cb, 10)
        self.create_subscription(
            Int32, 'grasp_executor_state', self.executor_state_cb, 10)
        self.create_subscription(
            Int32, 'grasp_result', self.grasp_result_cb, 10)
        self.create_subscription(
            Empty, 'shelf/skip_align', self.skip_align_cb, 10)
        self.create_subscription(
            Empty, 'shelf/preset_home', self.preset_home_cb, 10)
        # 深度停滞监测（诊断用）：订阅对齐深度，只记到达时刻
        self._depth_mon_sub = self.create_subscription(
            Image, '/camera/camera/aligned_depth_to_color/image_raw',
            self.depth_mon_cb, 1)
        # 关节状态（用于"臂稳定才规划下一动"）
        self.create_subscription(
            JointState, 'feedback/joint_states', self.joint_state_cb, 10)
        self.create_timer(0.2, self._joint_stability_update)

        self.state = self.IDLE
        self._layer = None
        self._latest_aruco = None
        self._last_box_time = 0.0
        self._last_grasp_pose_time = 0.0
        self._stable_ticks = 0
        self._last_box_key = None
        self._settle_logged = False
        self._executor_state = None
        self._executor_state_time = 0.0
        self._release_pending = False
        self._skip_align = False
        self._preset_home = False
        self._startup_home_sent = False
        self._home_sent = False
        self._home_done = False
        self._home_done_time = 0.0
        self._nominal_sent = False
        self._nominal_target_pose = None
        self._align_start = 0.0
        self._align_iter = 0
        self._align_sent = False
        self._align_no_marker_logged = False
        self._detect_start = 0.0
        self._detect_warn_logged = False
        self._depth_arrival = 0.0
        self._depth_mon_on = False
        self._depth_mon_start = 0.0
        self._depth_stalling = False
        self._depth_stall_count = 0
        self._depth_max_gap = 0.0
        self._depth_stall_accum = 0.0
        self._depth_stall_start = 0.0
        self._depth_stall_logged = 0.0
        self._joint_pos = None
        self._joint_prev = None
        self._joint_stable_ticks = 0
        self._joint_last_feedback = 0.0
        self._joint_stable = False
        self._align_stable_logged = 0.0
        self._latest_grasp_pt = None
        self._latest_grasp_pose = None
        self._table_z = None
        self._plausible_ticks = 0
        self._implausible_logged = False
        self._grasp_retries = 0
        self._last_grasp_result = None
        self._last_grasp_result_time = 0.0
        self._release_force_sent = False
        self._ret_home_sent = False
        self._ret_home_start = 0.0
        self._auto_retry_home = False
        self._grasp_trigger_time = 0.0
        self._grasp_fail_start = None
        self._place_start = 0.0
        self._state_log_time = 0.0
        self._busy_executor_logged = False
        self._home_quat = None

        self.create_timer(0.1, self.tick)
        self.get_logger().info(
            'Shelf workflow ready; waiting for /task_command (layer 1-3)')

    # ------------------------------------------------------------------
    # callbacks
    # ------------------------------------------------------------------
    def task_cmd_cb(self, msg):
        if self.state != self.IDLE:
            self.get_logger().warning(
                f'Busy in state {self.STATE_NAMES.get(self.state)}; '
                'task command ignored')
            return
        layer = int(msg.data)
        cfg = self._layer_cfg(layer)
        if cfg is None:
            self.get_logger().error(
                f'Unknown layer {layer}; configured layers: '
                f'{[l.get("layer") for l in self._cfg.get("layers", [])]}')
            return
        self._layer = cfg
        self._reset_cycle_vars()
        self.get_logger().info(
            f'Task received: layer={layer} aruco_id={cfg.get("aruco_id")} '
            f'layer_height={cfg.get("layer_height")} m')
        now = self.get_clock().now().nanoseconds * 1e-9
        if self._skip_align:
            self._detect_start = now
            self._detect_warn_logged = False
            self.get_logger().info(
                '[TEST] skip_align: entering WAIT_DETECT directly '
                '(skip nominal/align)')
            self._set_state(self.WAIT_DETECT)
        elif self.skip_nominal:
            self._align_start = now
            self._align_iter = 0
            self._align_sent = False
            self._align_no_marker_logged = False
            self.get_logger().info(
                '[TEST] skip_nominal: assuming coarse alignment done; '
                'entering ALIGN (aruco search)')
            self._set_state(self.ALIGN)
        else:
            self._set_state(self.NOMINAL_POSE)

    def release_cmd_cb(self, msg):
        del msg
        self._release_pending = True
        if self.state == self.WAIT_RELEASE_CMD:
            self._fire_release()
        else:
            self.get_logger().info(
                'Release command stored; will be applied when ready')

    def aruco_cb(self, msg):
        self._latest_aruco = (
            self.get_clock().now().nanoseconds * 1e-9, msg)

    def target_box_cb(self, msg):
        self._last_box_time = self.get_clock().now().nanoseconds * 1e-9
        p = msg.pose.position
        s = msg.scale
        cur = (float(p.x), float(p.y), float(p.z),
               float(s.x), float(s.y), float(s.z))
        prev = self._last_box_key
        if prev is not None:
            same = True
            for a, b in zip(cur[:3], prev[:3]):
                if abs(a - b) > self.box_stable_tol:
                    same = False
                    break
            if same:
                for a, b in zip(cur[3:], prev[3:]):
                    if abs(a - b) > self.box_stable_tol:
                        same = False
                        break
            if same:
                self._stable_ticks += 1
                return
        self._last_box_key = cur
        self._stable_ticks = 1

    def grasp_pose_cb(self, msg):
        self._latest_grasp_pt = (
            float(msg.pose.position.x), float(msg.pose.position.y),
            float(msg.pose.position.z))
        self._latest_grasp_pose = msg
        self._last_grasp_pose_time = self.get_clock().now().nanoseconds * 1e-9

    def depth_mon_cb(self, msg):
        del msg
        self._depth_arrival = self.get_clock().now().nanoseconds * 1e-9

    def joint_state_cb(self, msg):
        names = ['joint1', 'joint2', 'joint3', 'joint4',
                 'joint5', 'joint6', 'joint7']
        if self._joint_pos is None:
            self._joint_pos = [0.0] * 7
        for n, name in enumerate(names):
            if name in msg.name:
                idx = msg.name.index(name)
                if idx < len(msg.position):
                    self._joint_pos[n] = msg.position[idx]
        self._joint_last_feedback = self.get_clock().now().nanoseconds * 1e-9

    def _joint_stability_update(self):
        """0.2s 采样关节：delta<0.01rad 连续 3 次判定稳定。"""
        now = self.get_clock().now().nanoseconds * 1e-9
        if now - self._joint_last_feedback > 0.5:
            self._joint_stable = False
            self._joint_stable_ticks = 0
            return
        cur = self._joint_pos
        if cur is None:
            self._joint_stable = False
            return
        if self._joint_prev is None:
            self._joint_prev = list(cur)
            return
        delta = max(abs(a - b) for a, b in zip(cur, self._joint_prev))
        self._joint_prev = list(cur)
        if delta < 0.01:
            self._joint_stable_ticks += 1
        else:
            self._joint_stable_ticks = 0
        self._joint_stable = (self._joint_stable_ticks >= 3)

    def _joints_stable(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        if now - self._joint_last_feedback > 0.5:
            return False
        return self._joint_stable

    def _start_depth_monitor(self):
        self._depth_arrival = self.get_clock().now().nanoseconds * 1e-9
        self._depth_mon_on = True
        self._depth_mon_start = self._depth_arrival
        self._depth_stalling = False
        self._depth_stall_count = 0
        self._depth_max_gap = 0.0
        self._depth_stall_accum = 0.0
        self._depth_stall_logged = 0.0
        self.get_logger().info('[MON] depth monitor started (align done)')

    def _depth_monitor_tick(self, now):
        """监测深度流停滞：gap>3s 记录一次，恢复时记录时长。"""
        if not self._depth_mon_on or self._depth_arrival <= 0.0:
            return
        gap = now - self._depth_arrival
        if gap > self._depth_max_gap:
            self._depth_max_gap = gap
        if gap > 3.0:
            if not self._depth_stalling:
                self._depth_stalling = True
                self._depth_stall_count += 1
                self._depth_stall_start = now
                self.get_logger().warning(
                    f'[MON] depth stalled: {gap:.1f}s no frame '
                    f'(stall #{self._depth_stall_count})')
            elif (now - self._depth_stall_logged > 5.0):
                self._depth_stall_logged = now
                self.get_logger().warning(
                    f'[MON] depth still stalled: {gap:.1f}s')
        else:
            if self._depth_stalling:
                self._depth_stalling = False
                duration = now - self._depth_stall_start
                self._depth_stall_accum += duration
                self.get_logger().warning(
                    f'[MON] depth resumed after {duration:.1f}s '
                    f'(gap now {gap:.2f}s)')

    def _stop_depth_monitor(self):
        if not self._depth_mon_on:
            return
        self._depth_mon_on = False
        self.get_logger().info(
            f'[MON] depth monitor end: {self._depth_stall_count} stall(s), '
            f'max gap {self._depth_max_gap:.1f}s, '
            f'accum stalled {self._depth_stall_accum:.1f}s')

    def executor_state_cb(self, msg):
        self._executor_state = int(msg.data)
        self._executor_state_time = self.get_clock().now().nanoseconds * 1e-9

    def grasp_result_cb(self, msg):
        self._last_grasp_result = int(msg.data)
        self._last_grasp_result_time = (
            self.get_clock().now().nanoseconds * 1e-9)

    def skip_align_cb(self, msg):
        del msg
        self._skip_align = True
        self.get_logger().warn(
            '[TEST] skip_align set; will skip ALIGN when reached')

    def preset_home_cb(self, msg):
        del msg
        self._preset_home = True
        self.get_logger().warn(
            '[TEST] preset_home set; NOMINAL_POSE will use home position')

    # ------------------------------------------------------------------
    # helpers
    # ------------------------------------------------------------------
    def _set_state(self, state):
        if state == self.state:
            return
        self.get_logger().info(
            f'State: {self.STATE_NAMES.get(self.state)} -> '
            f'{self.STATE_NAMES.get(state)}')
        self.state = state
        if state == self.WAIT_DETECT:
            self._start_depth_monitor()
        elif state == self.IDLE:
            self._stop_depth_monitor()

    def _target_plausible(self):
        """抓取目标 z 合理性：相对点云桌面(aruco z) 在 [z_min, z_max] 上方。"""
        if self._latest_grasp_pt is None:
            return False
        if self._table_z is None:
            # 无桌面参考（如 skip_align 测试）时不按 z 过滤
            return True
        z = self._latest_grasp_pt[2]
        return (self._table_z + self.target_z_min <= z
                <= self._table_z + self.target_z_max)

    def _enter_home_fail(self, reason):
        """目标有效性不通过/抓取失败多次 → 回 home 位姿再回 IDLE。"""
        self.get_logger().error(
            f'{reason}; returning home (send /task_command to retry)')
        self._ret_home_sent = False
        self._ret_home_start = self.get_clock().now().nanoseconds * 1e-9
        self._set_state(self.RETURN_HOME)

    def _no_aruco_fallback(self, now):
        """对准找不到 aruco：预算内 → 回 home 后自动重跑完整流程；否则停。"""
        self._grasp_retries += 1
        if self._grasp_retries <= self.grasp_max_retries:
            self.get_logger().warning(
                f'No aruco marker for {self.align_give_up_timeout:.0f}s; '
                f'retry {self._grasp_retries}/{self.grasp_max_retries} — '
                f'returning home, then re-running the full task flow')
            self._auto_retry_home = True
            self._ret_home_sent = False
            self._ret_home_start = now
            self._set_state(self.RETURN_HOME)
        else:
            self.get_logger().error(
                f'No aruco marker after '
                f'{self.grasp_max_retries} retries; aborting task '
                f'(back to IDLE, send /task_command to retry)')
            self._reset_cycle_vars()
            self._set_state(self.IDLE)

    def _handle_grasp_failure(self, now):
        """抓取失败（grasp_result=0）：若 executor 停在 WAIT_RELEASE（中途失败后
        举着空爪回 home），先 release_force 让它回 IDLE；再等 IDLE 后重试或回 home。"""
        if self._executor_state == self.EXECUTOR_WAIT_RELEASE:
            if not self._release_force_sent:
                self._release_force_sent = True
                self.release_force_pub.publish(Empty())
                self.get_logger().info(
                    '[GRASP] failed grasp; release_force to idle')
            return
        if self._executor_state != self.EXECUTOR_IDLE:
            # executor 还在回 home / 过渡中 → 等它回 IDLE
            return
        self._retry_detect(now, 'Grasp failed (no object / unreachable)')

    def _retry_detect(self, now, reason):
        """一轮失败（抓取失败/检测超时）：若还有次数则回 home(executor 已回初始位)
        后重跑完整流程（重新对准 ALIGN → 再检测/尝试夹取）；否则放弃回 home。"""
        self._grasp_retries += 1
        if self._grasp_retries <= self.grasp_max_retries:
            self.get_logger().warning(
                f'{reason}; retry {self._grasp_retries}/'
                f'{self.grasp_max_retries} — re-aligning, then retrying the cycle')
            self._detect_start = now
            self._detect_warn_logged = False
            self._plausible_ticks = 0
            self._implausible_logged = False
            self._align_start = now
            self._align_iter = 0
            self._align_sent = False
            self._align_no_marker_logged = False
            self._set_state(self.ALIGN)
        else:
            self._enter_home_fail(
                '目标有效性不通过（多次失败）')

    def _reset_cycle_vars(self):
        self._release_pending = False
        self._home_sent = False
        self._home_done = False
        self._home_done_time = 0.0
        self._nominal_sent = False
        self._nominal_target_pose = None
        self._align_start = 0.0
        self._align_iter = 0
        self._align_sent = False
        self._align_no_marker_logged = False
        self._detect_warn_logged = False
        self._stable_ticks = 0
        self._last_box_key = None
        self._settle_logged = False
        self._grasp_fail_start = None
        self._state_log_time = 0.0
        self._busy_executor_logged = False
        self._latest_grasp_pt = None
        self._latest_grasp_pose = None
        self._table_z = None
        self._plausible_ticks = 0
        self._implausible_logged = False
        self._grasp_retries = 0
        self._last_grasp_result = None
        self._last_grasp_result_time = 0.0
        self._release_force_sent = False
        self._ret_home_sent = False
        self._ret_home_start = 0.0
        self._auto_retry_home = False

    def _layer_cfg(self, layer):
        for entry in self._cfg.get('layers', []):
            if int(entry.get('layer')) == int(layer):
                return entry
        return None

    def _cfg_num(self, key, default):
        value = self._cfg.get(key, default)
        return float(value) if value is not None else float(default)

    def _lookup_matrix(self, target_frame, source_frame):
        try:
            t = self.tf_buffer.lookup_transform(
                target_frame, source_frame, rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=1.0))
        except TransformException as exc:
            self.get_logger().warn(
                f'TF {target_frame}<-{source_frame} unavailable: {exc}',
                throttle_duration_sec=5.0)
            return None
        q = t.transform.rotation
        rot = R.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
        m = np.eye(4)
        m[:3, :3] = rot
        m[:3, 3] = [
            t.transform.translation.x,
            t.transform.translation.y,
            t.transform.translation.z,
        ]
        return m

    @staticmethod
    def _rot_from_view(view):
        z = view / (np.linalg.norm(view) + 1e-12)
        up = np.array([0.0, 0.0, 1.0])
        x = np.cross(z, up)
        nx = np.linalg.norm(x)
        if nx < 1e-6:
            x = np.array([1.0, 0.0, 0.0])
        else:
            x = x / nx
        y = np.cross(z, x)
        return np.column_stack([x, y, z])

    def _tcp_pose_from_camera(self, cam_pos, view_dir):
        """Convert a target camera pose (pos + viewing direction) in
        base frame to a target TCP Pose in base frame."""
        t_base_cam = self._lookup_matrix(self.base_frame, self.camera_frame)
        t_base_tcp = self._lookup_matrix(self.base_frame, self.end_effector)
        if t_base_cam is None or t_base_tcp is None:
            return None
        t_cam_tcp = np.linalg.inv(t_base_cam) @ t_base_tcp
        t_target_cam = np.eye(4)
        t_target_cam[:3, :3] = self._rot_from_view(view_dir)
        t_target_cam[:3, 3] = np.asarray(cam_pos, dtype=float)
        t_target_tcp = t_target_cam @ t_cam_tcp

        pose = Pose()
        pose.position.x = float(t_target_tcp[0, 3])
        pose.position.y = float(t_target_tcp[1, 3])
        pose.position.z = float(t_target_tcp[2, 3])
        quat = R.from_matrix(t_target_tcp[:3, :3]).as_quat()
        pose.orientation.x = float(quat[0])
        pose.orientation.y = float(quat[1])
        pose.orientation.z = float(quat[2])
        pose.orientation.w = float(quat[3])
        return pose

    def _view_pose_from_marker(self, marker_pt):
        """Viewing pose: the TCP lands TCP_BACK_OFFSET toward the arm and
        TCP_HEIGHT_OFFSET above the marker; the TCP orientation makes the
        wrist camera optical axis face the marker with roll constrained to
        world-up (no wrist flip)."""
        v = marker_pt.copy()
        v[2] = 0.0
        nv = np.linalg.norm(v)
        dirh = v / nv if nv > 1e-6 else np.array([1.0, 0.0, 0.0])
        tcp_pos = (marker_pt
                   - self.TCP_BACK_OFFSET * dirh
                   + np.array([0.0, 0.0, self.TCP_HEIGHT_OFFSET]))

        view_dir = marker_pt - tcp_pos
        nv_dir = float(np.linalg.norm(view_dir))
        t_base_cam = self._lookup_matrix(
            self.base_frame, self.camera_frame)
        t_base_tcp = self._lookup_matrix(
            self.base_frame, self.end_effector)
        if (nv_dir > 1e-9 and t_base_cam is not None
                and t_base_tcp is not None):
            # 相机 look-at（滚转约束到世界向上，不翻转），再乘相机→TCP 安装旋转
            R_base_cam = self._rot_from_view(view_dir)
            t_tcp_cam = np.linalg.inv(t_base_tcp) @ t_base_cam
            R_cam_tcp = t_tcp_cam[:3, :3].T
            R_tcp = R_base_cam @ R_cam_tcp
            quat = R.from_matrix(R_tcp).as_quat()
            pose = Pose()
            pose.position.x = float(tcp_pos[0])
            pose.position.y = float(tcp_pos[1])
            pose.position.z = float(tcp_pos[2])
            pose.orientation.x = float(quat[0])
            pose.orientation.y = float(quat[1])
            pose.orientation.z = float(quat[2])
            pose.orientation.w = float(quat[3])
            return pose

        # 回退固定 home 姿态（TF 不可用时）
        pose = Pose()
        pose.position.x = float(tcp_pos[0])
        pose.position.y = float(tcp_pos[1])
        pose.position.z = float(tcp_pos[2])
        qx, qy, qz, qw = self._fixed_tcp_quat()
        pose.orientation.x = float(qx)
        pose.orientation.y = float(qy)
        pose.orientation.z = float(qz)
        pose.orientation.w = float(qw)
        return pose

    def _fixed_tcp_quat(self):
        if self._home_quat is None:
            t_base_tcp = self._lookup_matrix(
                self.base_frame, self.end_effector)
            if t_base_tcp is not None:
                quat = R.from_matrix(t_base_tcp[:3, :3]).as_quat()
                self._home_quat = tuple(float(q) for q in quat)
                self.get_logger().info(
                    'Fixed viewing orientation set from current tcp pose')
            else:
                self._home_quat = self.HOME_TCP_QUAT
                self.get_logger().warn(
                    'TF unavailable; using hardcoded home quaternion')
        return self._home_quat

    def _nominal_tcp_pose(self):
        sx = self._cfg_num('shelf_x', 0.45)
        sy = self._cfg_num('shelf_y', 0.0)
        bh = self._cfg_num('base_height', 0.0)
        h = float(self._layer['layer_height']) - bh
        marker_pt = np.array([sx, sy, h])
        return self._view_pose_from_marker(marker_pt)

    def _fresh_target_marker(self, now):
        if self._layer is None or self._latest_aruco is None:
            return None
        stamp, msg = self._latest_aruco
        if now - stamp > self.marker_fresh_timeout:
            return None
        target_id = int(self._layer.get('aruco_id'))
        for marker in msg.markers:
            if int(marker.marker_id) == target_id:
                return marker, msg.header.frame_id
        return None

    def _align_tcp_pose(self, marker, detection_frame):
        # 精对准位姿：相机放在 aruco 前方 align_dist(0.4m) 处，相机 z 轴（光轴）
        # 指向 aruco 坐标位置（使 aruco 中心投影到画面中心）。
        # 只用 aruco 的【位置】，不用其朝向；不要求光轴与码平面平行或垂直。
        del detection_frame
        t_base_cam = self._lookup_matrix(self.base_frame, self.camera_frame)
        t_base_tcp = self._lookup_matrix(self.base_frame, self.end_effector)
        if t_base_cam is None or t_base_tcp is None:
            return None
        p_cam = np.array([
            marker.pose.position.x,
            marker.pose.position.y,
            marker.pose.position.z,
            1.0,
        ])
        a_base = (t_base_cam @ p_cam)[:3]
        # aruco 平贴桌面 → 其 base z 即桌面参考高度（用于抓取目标 z 合理性校验）
        self._table_z = float(a_base[2])
        cam0 = t_base_cam[:3, 3]
        d = a_base - cam0
        nd = float(np.linalg.norm(d))
        if nd < 1e-6:
            return None
        dirv = d / nd
        cam_pos = a_base - self.align_dist * dirv
        R_base_cam = self._rot_from_view(dirv)
        t_tcp_cam = np.linalg.inv(t_base_tcp) @ t_base_cam
        R_cam_tcp = t_tcp_cam[:3, :3].T
        R_tcp = R_base_cam @ R_cam_tcp
        tcp_pos = cam_pos - R_tcp @ t_tcp_cam[:3, 3]
        quat = R.from_matrix(R_tcp).as_quat()
        pose = Pose()
        pose.position.x = float(tcp_pos[0])
        pose.position.y = float(tcp_pos[1])
        pose.position.z = float(tcp_pos[2])
        pose.orientation.x = float(quat[0])
        pose.orientation.y = float(quat[1])
        pose.orientation.z = float(quat[2])
        pose.orientation.w = float(quat[3])
        return pose

    def _align_verified(self, marker, detection_frame):
        """精对准成功判定：aruco 距相机约 align_dist 米，且其中心投影到画面中心。"""
        del detection_frame
        x = float(marker.pose.position.x)
        y = float(marker.pose.position.y)
        z = float(marker.pose.position.z)
        if z <= 0.05:
            return False
        dist = float(np.sqrt(x * x + y * y + z * z))
        if abs(dist - self.align_dist) > self.align_dist_tol:
            return False
        if (abs(x / z) > self.align_center_tol
                or abs(y / z) > self.align_center_tol):
            return False
        return True

    def _tcp_position_error(self, pose):
        t_base_tcp = self._lookup_matrix(self.base_frame, self.end_effector)
        if t_base_tcp is None:
            return None
        actual = t_base_tcp[:3, 3]
        target = np.array([
            pose.position.x, pose.position.y, pose.position.z])
        return float(np.linalg.norm(actual - target))

    def _publish_align_target(self, pose):
        header = Header()
        header.frame_id = self.base_frame
        header.stamp = self.get_clock().now().to_msg()

        sphere = Marker()
        sphere.header = header
        sphere.ns = 'shelf_align_target'
        sphere.id = 0
        sphere.type = Marker.SPHERE
        sphere.action = Marker.ADD
        sphere.pose = pose
        sphere.scale.x = 0.03
        sphere.scale.y = 0.03
        sphere.scale.z = 0.03
        sphere.color = ColorRGBA(r=0.2, g=0.6, b=1.0, a=0.9)
        self.align_target_pub.publish(sphere)

        arrow = Marker()
        arrow.header = header
        arrow.ns = 'shelf_align_target'
        arrow.id = 1
        arrow.type = Marker.ARROW
        arrow.action = Marker.ADD
        arrow.pose = pose
        arrow.scale.x = 0.08
        arrow.scale.y = 0.01
        arrow.scale.z = 0.02
        arrow.color = ColorRGBA(r=0.2, g=0.6, b=1.0, a=0.9)
        self.align_target_pub.publish(arrow)

    # ------------------------------------------------------------------
    # state machine
    # ------------------------------------------------------------------
    def tick(self):
        now = self.get_clock().now().nanoseconds * 1e-9

        # 启动一次性归位（等 move_group 就绪后发一次，之后不再归位）
        if not self._startup_home_sent:
            if self.arm.action.server_is_ready():
                self._startup_home_sent = True
                self.get_logger().info(
                    'Startup: moving to home (one-time)')
                self.arm.move_to_joints(
                    self.home_joints,
                    velocity_scaling=self.velocity_scaling)

        if self.state == self.IDLE:
            return

        self._depth_monitor_tick(now)

        if self.state == self.NOMINAL_POSE:
            if self._executor_state != self.EXECUTOR_IDLE:
                if not self._busy_executor_logged:
                    self.get_logger().warn(
                        'grasp_executor not IDLE; waiting before moving arm')
                    self._busy_executor_logged = True
                return
            if not self._nominal_sent:
                if self._preset_home:
                    self._nominal_target_pose = None
                    self.get_logger().info(
                        '[TEST] preset_home: moving to home joints '
                        '(preset position)')
                    self.arm.move_to_joints(
                        self.home_joints,
                        velocity_scaling=self.velocity_scaling)
                else:
                    pose = self._nominal_tcp_pose()
                    if pose is None:
                        return
                    self._nominal_target_pose = pose
                    self.get_logger().info(
                        'Moving to nominal shelf-view pose...')
                    self._publish_align_target(pose)
                    self.arm.move_to_pose(
                        pose, self.base_frame,
                        velocity_scaling=self.velocity_scaling)
                self._nominal_sent = True
                return
            if not self.arm.is_done():
                return
            self._nominal_sent = False
            if self.arm.success:
                if self._skip_align:
                    self._detect_start = now
                    self._detect_warn_logged = False
                    self.get_logger().info(
                        '[TEST] Skipping ALIGN; entering WAIT_DETECT')
                    self._set_state(self.WAIT_DETECT)
                    return
                self._align_start = now
                self._align_iter = 0
                self._align_sent = False
                self._align_no_marker_logged = False
                self.get_logger().info(
                    'Nominal pose reached; searching aruco marker')
                self._set_state(self.ALIGN)
            else:
                if self._nominal_target_pose is not None:
                    p = self._nominal_target_pose.position
                    self.get_logger().error(
                        f'Nominal pose move failed; target tcp='
                        f'({p.x:.3f},{p.y:.3f},{p.z:.3f}). Adjust shelf_x/'
                        f'shelf_y/layer_height/standoff, then send '
                        f'/task_command to retry')
                else:
                    self.get_logger().error(
                        'Nominal move failed (nominal=home); '
                        'check home reachability')
                self._reset_cycle_vars()
                self._set_state(self.IDLE)
            return

        if self.state == self.ALIGN:
            if self._skip_align:
                self._detect_start = now
                self._detect_warn_logged = False
                self.get_logger().info(
                    '[TEST] Skipping ALIGN (in ALIGN); entering WAIT_DETECT')
                self._set_state(self.WAIT_DETECT)
                return
            if self._executor_state != self.EXECUTOR_IDLE:
                if not self._busy_executor_logged:
                    self.get_logger().warn(
                        'grasp_executor not IDLE; pausing alignment')
                    self._busy_executor_logged = True
                return
            found = self._fresh_target_marker(now)
            if found is None:
                if (now - self._align_start > self.aruco_timeout
                        and not self._align_no_marker_logged):
                    self.get_logger().warn(
                        f'No aruco marker (id='
                        f'{self._layer.get("aruco_id")}) in view within '
                        f'{self.aruco_timeout:.1f}s; staying at nominal '
                        f'pose (check shelf_x/standoff or intervene)')
                    self._align_no_marker_logged = True
                if now - self._align_start > self.align_give_up_timeout:
                    self._no_aruco_fallback(now)
                return
            self._align_no_marker_logged = False
            self._align_start = now
            marker, frame_id = found
            if self._align_verified(marker, frame_id):
                x = float(marker.pose.position.x)
                y = float(marker.pose.position.y)
                z = float(marker.pose.position.z)
                dist = float(np.sqrt(x * x + y * y + z * z))
                self.get_logger().info(
                    f'Aligned to marker (dist={dist:.3f} m, '
                    f'center=({x / z:.3f},{y / z:.3f}))')
                self._set_state(self.WAIT_DETECT)
                self._detect_start = now
                self._detect_warn_logged = False
                return
            pose = self._align_tcp_pose(marker, frame_id)
            if pose is None:
                return
            error = self._tcp_position_error(pose)
            if not self._align_sent:
                # 等臂真正停稳再规划/发送下一动，避免 MoveIt 从中间状态规划导致
                # 轨迹起点与实际不符（Invalid Trajectory: start point deviates）
                if not self._joints_stable():
                    now_s = self.get_clock().now().nanoseconds * 1e-9
                    if now_s - self._align_stable_logged > 3.0:
                        self._align_stable_logged = now_s
                        self.get_logger().info(
                            '[ALIGN] waiting for arm joints to settle '
                            'before next move')
                    return
                self.get_logger().info(
                    f'Refining alignment (iter={self._align_iter + 1}, '
                    f'current error={error if error is not None else -1:.3f} m)')
                self._publish_align_target(pose)
                self.arm.move_to_pose(
                    pose, self.base_frame,
                    velocity_scaling=self.velocity_scaling)
                self._align_sent = True
                return
            if not self.arm.is_done():
                return
            self._align_sent = False
            self._align_iter += 1
            if self.arm.success:
                if self._align_iter >= self.align_max_iter:
                    self.get_logger().info(
                        'Align refinement iterations done; proceeding')
                    self._set_state(self.WAIT_DETECT)
                    self._detect_start = now
                    self._detect_warn_logged = False
            else:
                p = pose.position
                self.get_logger().error(
                    f'Align move failed; target tcp='
                    f'({p.x:.3f},{p.y:.3f},{p.z:.3f}). Send '
                    f'/task_command to retry')
                self._reset_cycle_vars()
                self._set_state(self.IDLE)
            return

        if self.state == self.WAIT_DETECT:
            if now - self._detect_start < self.align_settle_time:
                if not self._settle_logged:
                    self._settle_logged = True
                    self.get_logger().info(
                        f'Waiting {self.align_settle_time:.1f}s for camera '
                        'to settle after alignment...')
                return
            fresh_box = now - self._last_box_time <= 6.0
            fresh_pose = now - self._last_grasp_pose_time <= 6.0
            plausible = self._target_plausible()
            if fresh_box and fresh_pose and plausible:
                self._plausible_ticks += 1
            else:
                self._plausible_ticks = 0
                if fresh_box and fresh_pose and not plausible \
                        and not self._implausible_logged:
                    self._implausible_logged = True
                    self.get_logger().warning(
                        f'[DETECT] implausible target z='
                        f'{(self._latest_grasp_pt[2] if self._latest_grasp_pt else 0):.3f} '
                        f'(table_z={self._table_z}); waiting for valid target')
            if self._plausible_ticks >= 3:
                self.get_logger().info(
                    f'Target plausible (z within band, x{self._plausible_ticks}); '
                    'triggering grasp')
                self._set_state(self.TRIGGER_GRASP)
                return
            if (now - self._detect_start > self.detect_timeout
                    and not self._detect_warn_logged):
                self.get_logger().warn(
                    f'No fresh yolo detection within '
                    f'{self.detect_timeout:.1f}s; object may be absent '
                    f'or out of view')
                self._detect_warn_logged = True
            if now - self._detect_start > self.detect_give_up_timeout:
                self._retry_detect(now, '目标有效性不通过（等待超时）')
                return
            return

        if self.state == self.TRIGGER_GRASP:
            # Fix A: 先把"确认的抓取位姿"发给 executor（防触发时锁到漂移 pose），再触发
            if self._latest_grasp_pose is not None:
                self.grasp_cmd_pub.publish(self._latest_grasp_pose)
            self.grasp_start_pub.publish(Empty())
            self._grasp_trigger_time = now
            self._grasp_fail_start = None
            self._last_grasp_result = None
            self._release_force_sent = False
            self._state_log_time = 0.0
            self.get_logger().info('Published /manual_grasp_start')
            self._set_state(self.GRASPING)
            return

        if self.state == self.GRASPING:
            if self._executor_state == self.EXECUTOR_WAIT_RELEASE:
                # executor 到 WAIT_RELEASE：用 grasp_result 区分成功/失败
                if (self._last_grasp_result is not None
                        and self._last_grasp_result <= 0):
                    # 0 = 未执行/中途失败（失败也会回 home 到 WAIT_RELEASE）→ 失败处理
                    self._handle_grasp_failure(now)
                    return
                self.get_logger().info(
                    'Grasp complete; arm returned to initial position, '
                    'waiting for /release_command')
                self._set_state(self.WAIT_RELEASE_CMD)
                return
            self._log_executor_state(now)
            # grasp_result 0（校验不可达/前段失败）→ 失败处理
            if (self._last_grasp_result is not None
                    and self._last_grasp_result <= 0):
                self._handle_grasp_failure(now)
                return
            # 未收到 grasp_result：executor 在校验(state 0)/执行中 → 继续等
            if now - self._grasp_trigger_time > self.grasp_give_up_timeout:
                self.get_logger().error(
                    f'Grasp not finished within '
                    f'{self.grasp_give_up_timeout:.0f}s; aborting task '
                    f'(send /task_command to retry)')
                self._reset_cycle_vars()
                self._set_state(self.IDLE)
            return

        if self.state == self.WAIT_RELEASE_CMD:
            if self._release_pending:
                self._fire_release()
            return

        if self.state == self.PLACING:
            if self._executor_state == self.EXECUTOR_IDLE:
                self.get_logger().info(
                    'Place sequence finished; task complete, back to IDLE')
                self._layer = None
                self._set_state(self.IDLE)
                return
            self._log_executor_state(now)
            if now - self._place_start > self.place_give_up_timeout:
                self.get_logger().error(
                    f'Place not finished within '
                    f'{self.place_give_up_timeout:.0f}s; aborting task '
                    f'(send /task_command to retry)')
                self._reset_cycle_vars()
                self._set_state(self.IDLE)
            return

        if self.state == self.RETURN_HOME:
            if self._executor_state != self.EXECUTOR_IDLE:
                if now - self._ret_home_start > self.ret_home_exec_wait:
                    self.get_logger().warning(
                        'executor busy too long; forcing IDLE')
                    self._reset_cycle_vars()
                    self._set_state(self.IDLE)
                return
            if not self._ret_home_sent:
                self.get_logger().info(
                    'Returning home before entering IDLE')
                self.arm.move_to_joints(
                    self.home_joints,
                    velocity_scaling=self.velocity_scaling)
                self._ret_home_sent = True
                return
            if not self.arm.is_done():
                if now - self._ret_home_start > self.ret_home_timeout:
                    self.get_logger().warning(
                        'home return timeout; forcing IDLE')
                    self._reset_cycle_vars()
                    self._set_state(self.IDLE)
                return
            retry_home = self._auto_retry_home
            self._auto_retry_home = False
            saved_retries = self._grasp_retries
            self._reset_cycle_vars()
            if retry_home:
                # 无 aruco 回退：回 home 后自动重跑完整流程（保留重试计数）
                self._grasp_retries = saved_retries
                now_s = self.get_clock().now().nanoseconds * 1e-9
                if self._skip_align:
                    self._detect_start = now_s
                    self._detect_warn_logged = False
                    self.get_logger().info(
                        '[TEST] skip_align: entering WAIT_DETECT (auto-retry)')
                    self._set_state(self.WAIT_DETECT)
                elif self.skip_nominal:
                    self._align_start = now_s
                    self._align_iter = 0
                    self._align_sent = False
                    self._align_no_marker_logged = False
                    self.get_logger().info(
                        '[TEST] skip_nominal: entering ALIGN (auto-retry)')
                    self._set_state(self.ALIGN)
                else:
                    self.get_logger().info(
                        'Auto-retry: moving to nominal pose')
                    self._set_state(self.NOMINAL_POSE)
            else:
                self._set_state(self.IDLE)
            return

    def _fire_release(self):
        self._release_pending = False
        self.release_pub.publish(Empty())
        self._place_start = self.get_clock().now().nanoseconds * 1e-9
        self._state_log_time = 0.0
        self.get_logger().info('Published /manual_release (place sequence)')
        self._set_state(self.PLACING)

    def _log_executor_state(self, now):
        if now - self._state_log_time < 1.0:
            return
        self._state_log_time = now
        name = self.EXECUTOR_STATE_NAMES.get(
            self._executor_state, str(self._executor_state))
        self.get_logger().info(
            f'executor state: {self._executor_state} ({name})')


def main():
    rclpy.init()
    node = ShelfWorkflowNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
