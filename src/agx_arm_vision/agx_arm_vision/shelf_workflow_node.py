#!/usr/bin/env python3

import os

import numpy as np
import rclpy
import yaml
from aruco_opencv_msgs.msg import ArucoDetection
from geometry_msgs.msg import Pose, PoseStamped
from rclpy.node import Node
from scipy.spatial.transform import Rotation as R
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
    HOME_CHECK = 1
    NOMINAL_POSE = 2
    ALIGN = 3
    WAIT_DETECT = 4
    TRIGGER_GRASP = 5
    GRASPING = 6
    WAIT_RELEASE_CMD = 7
    PLACING = 8

    STATE_NAMES = {
        0: 'IDLE', 1: 'HOME_CHECK', 2: 'NOMINAL_POSE', 3: 'ALIGN',
        4: 'WAIT_DETECT', 5: 'TRIGGER_GRASP', 6: 'GRASPING',
        7: 'WAIT_RELEASE_CMD', 8: 'PLACING',
    }

    EXECUTOR_STATE_NAMES = {
        0: 'IDLE', 1: 'OPEN_GRIPPER', 2: 'MOVE_TO_TARGET',
        3: 'WAIT_REACH', 4: 'CLOSE_GRIPPER', 5: 'MOVE_HOME',
        6: 'WAIT_RELEASE', 7: 'MOVE_TO_PLACE_ABOVE',
        8: 'LOWER_TO_PLACE', 9: 'PLACE_OPEN', 10: 'PLACE_LIFT',
    }

    EXECUTOR_IDLE = 0
    EXECUTOR_WAIT_RELEASE = 6

    ALIGN_DIST_EPS = 0.42#for test remember to change it back to 0.02!!!!在rviz测试时临时使用！！！

    # TCP-referenced viewing pose offsets: the TCP lands TCP_BACK_OFFSET
    # toward the arm and TCP_HEIGHT_OFFSET above the marker.
    TCP_BACK_OFFSET = 0.05
    TCP_HEIGHT_OFFSET = 0.1
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
        self.declare_parameter('align_max_iter', 2)
        self.declare_parameter('settle_after_home', 1.0)
        self.declare_parameter('marker_fresh_timeout', 1.0)
        self.declare_parameter(
            'home_joints',
            [-1.751, -0.342, 1.656, 1.036, 0.360, 0.074, 1.570])

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
        self.align_max_iter = self.get_parameter('align_max_iter').value
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
            Empty, '/manual_grasp_start', 10)
        self.release_pub = self.create_publisher(
            Empty, '/manual_release', 10)
        self.align_target_pub = self.create_publisher(
            Marker, '/shelf/align_target', 10)

        self.create_subscription(
            Int32, '/task_command', self.task_cmd_cb, 10)
        self.create_subscription(
            Empty, '/release_command', self.release_cmd_cb, 10)
        self.create_subscription(
            ArucoDetection, '/aruco_detections', self.aruco_cb, 10)
        self.create_subscription(
            Marker, '/yolo/target_box', self.target_box_cb, 10)
        self.create_subscription(
            PoseStamped, '/grasp_pose', self.grasp_pose_cb, 10)
        self.create_subscription(
            Int32, '/grasp_executor_state', self.executor_state_cb, 10)

        self.state = self.IDLE
        self._layer = None
        self._latest_aruco = None
        self._last_box_time = 0.0
        self._last_grasp_pose_time = 0.0
        self._executor_state = None
        self._executor_state_time = 0.0
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
        self._detect_start = 0.0
        self._detect_warn_logged = False
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
        self._set_state(self.HOME_CHECK)

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
        del msg
        self._last_box_time = self.get_clock().now().nanoseconds * 1e-9

    def grasp_pose_cb(self, msg):
        del msg
        self._last_grasp_pose_time = self.get_clock().now().nanoseconds * 1e-9

    def executor_state_cb(self, msg):
        self._executor_state = int(msg.data)
        self._executor_state_time = self.get_clock().now().nanoseconds * 1e-9

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
        self._grasp_fail_start = None
        self._state_log_time = 0.0
        self._busy_executor_logged = False

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
        """Fixed home orientation viewing pose: the TCP lands
        TCP_BACK_OFFSET toward the arm and TCP_HEIGHT_OFFSET above the
        marker, keeping the home TCP orientation."""
        v = marker_pt.copy()
        v[2] = 0.0
        nv = np.linalg.norm(v)
        dirh = v / nv if nv > 1e-6 else np.array([1.0, 0.0, 0.0])
        tcp_pos = (marker_pt
                   - self.TCP_BACK_OFFSET * dirh
                   + np.array([0.0, 0.0, self.TCP_HEIGHT_OFFSET]))
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
        t_base_cam = self._lookup_matrix(self.base_frame, detection_frame)
        if t_base_cam is None:
            return None
        p_cam = np.array([
            marker.pose.position.x,
            marker.pose.position.y,
            marker.pose.position.z,
            1.0,
        ])
        p_base = t_base_cam @ p_cam
        return self._view_pose_from_marker(p_base[:3])

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

        if self.state == self.IDLE:
            return

        if self.state == self.HOME_CHECK:
            if not self._home_sent:
                self._home_sent = True
                self.get_logger().info(
                    'Moving to home (initial pose)')
                self.arm.move_to_joints(
                    self.home_joints,
                    velocity_scaling=self.velocity_scaling)
                return
            if not self.arm.is_done():
                return
            if not self.arm.success:
                self.get_logger().error(
                    'Home move failed; send /task_command to retry')
                self._reset_cycle_vars()
                self._set_state(self.IDLE)
                return
            if not self._home_done:
                self._home_done = True
                self._home_done_time = now
                self.get_logger().info('Home motion completed')
                return
            if now - self._home_done_time >= self.settle_after_home:
                self._set_state(self.NOMINAL_POSE)
            return

        if self.state == self.NOMINAL_POSE:
            if self._executor_state != self.EXECUTOR_IDLE:
                if not self._busy_executor_logged:
                    self.get_logger().warn(
                        'grasp_executor not IDLE; waiting before moving arm')
                    self._busy_executor_logged = True
                return
            if not self._nominal_sent:
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
                self._align_start = now
                self._align_iter = 0
                self._align_sent = False
                self._align_no_marker_logged = False
                self.get_logger().info(
                    'Nominal pose reached; searching aruco marker')
                self._set_state(self.ALIGN)
            else:
                p = self._nominal_target_pose.position
                self.get_logger().error(
                    f'Nominal pose move failed; target tcp='
                    f'({p.x:.3f},{p.y:.3f},{p.z:.3f}). Adjust shelf_x/'
                    f'shelf_y/layer_height/standoff, then send '
                    f'/task_command to retry')
                self._reset_cycle_vars()
                self._set_state(self.IDLE)
            return

        if self.state == self.ALIGN:
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
                    self.get_logger().error(
                        f'No aruco marker for '
                        f'{self.align_give_up_timeout:.0f}s; aborting task '
                        f'(back to IDLE, send /task_command to retry)')
                    self._reset_cycle_vars()
                    self._set_state(self.IDLE)
                return
            self._align_no_marker_logged = False
            self._align_start = now
            marker, frame_id = found
            pose = self._align_tcp_pose(marker, frame_id)
            if pose is None:
                return
            error = self._tcp_position_error(pose)
            if error is not None and error < self.ALIGN_DIST_EPS:
                self.get_logger().info(
                    f'Aligned to marker (tcp error={error:.3f} m)')
                self._set_state(self.WAIT_DETECT)
                self._detect_start = now
                self._detect_warn_logged = False
                return
            if not self._align_sent:
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
            fresh_box = now - self._last_box_time <= 1.0
            fresh_pose = now - self._last_grasp_pose_time <= 1.0
            if fresh_box and fresh_pose:
                self.get_logger().info(
                    'Target detected by yolo; triggering grasp')
                self._set_state(self.TRIGGER_GRASP)
            elif (now - self._detect_start > self.detect_timeout
                    and not self._detect_warn_logged):
                self.get_logger().warn(
                    f'No fresh yolo detection within '
                    f'{self.detect_timeout:.1f}s; object may be absent '
                    f'or out of view')
                self._detect_warn_logged = True
            return

        if self.state == self.TRIGGER_GRASP:
            self.grasp_start_pub.publish(Empty())
            self._grasp_trigger_time = now
            self._grasp_fail_start = None
            self._state_log_time = 0.0
            self.get_logger().info('Published /manual_grasp_start')
            self._set_state(self.GRASPING)
            return

        if self.state == self.GRASPING:
            if self._executor_state == self.EXECUTOR_WAIT_RELEASE:
                self.get_logger().info(
                    'Grasp complete; arm returned to initial position, '
                    'waiting for /release_command')
                self._set_state(self.WAIT_RELEASE_CMD)
                return
            self._log_executor_state(now)
            if self._executor_state == self.EXECUTOR_IDLE:
                if self._grasp_fail_start is None:
                    self._grasp_fail_start = now
                elif now - self._grasp_fail_start > self.grasp_fail_timeout:
                    self.get_logger().error(
                        'Grasp failed (executor back to IDLE); aborting '
                        'task (send /task_command to retry)')
                    self._reset_cycle_vars()
                    self._set_state(self.IDLE)
                    return
            else:
                self._grasp_fail_start = None
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
