#!/usr/bin/env python3

import rclpy
from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from scipy.spatial.transform import Rotation as R
from sensor_msgs.msg import JointState, PointCloud2
from std_msgs.msg import Bool, Int32
from std_msgs.msg import Empty as EmptyMsg
from std_srvs.srv import Empty as EmptySrv
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker

from agx_arm_vision.gripper_client import GripperClient
from agx_arm_vision.moveit2_local import MoveIt2


class GraspExecutor(Node):
    IDLE = 0
    OPEN_GRIPPER = 1
    MOVE_TO_TARGET = 2
    WAIT_REACH = 3
    CLOSE_GRIPPER = 4
    MOVE_HOME = 5
    WAIT_RELEASE = 6
    MOVE_TO_PLACE_ABOVE = 7
    LOWER_TO_PLACE = 8
    PLACE_OPEN = 9
    PLACE_LIFT = 10

    BOX_PAIR_WINDOW = 2.0

    def __init__(self):
        super().__init__('grasp_executor')
        self.declare_parameter('base_link', 'base_link')
        self.declare_parameter('end_effector_link', 'tcp_link')
        self.declare_parameter('arm_group', 'arm')
        self.declare_parameter('gripper_joint', 'gripper')
        self.declare_parameter('gripper_open', 0.1)
        self.declare_parameter('gripper_closed', 0.0)
        self.declare_parameter('gripper_width_tolerance', 0.002)
        self.declare_parameter('gripper_timeout', 3.0)
        self.declare_parameter('target_z_offset', 0.0)
        self.declare_parameter('require_target_box', True)
        self.declare_parameter('box_wait_timeout', 1.0)
        self.declare_parameter('force_threshold', 1.5)
        self.declare_parameter('reach_tolerance', 0.03)
        self.declare_parameter('reach_timeout', 45.0)
        self.declare_parameter('pre_approach_distance', 0.10)
        self.declare_parameter('velocity_scaling', 0.1)
        self.declare_parameter('idle_clear_interval', 10.0)
        self.declare_parameter('insert_settle', 0.3)
        self.declare_parameter('rebuild_timeout', 2.5)
        self.declare_parameter('box_apply_timeout', 5.0)
        self.declare_parameter('box_remove_timeout', 3.0)
        self.declare_parameter('place_pose_topic', 'place_pose')
        self.declare_parameter('place_cloud_topic', 'place/points_filtered')
        self.declare_parameter(
            'place_filtered_cloud_topic', 'place_filtered_cloud')
        self.declare_parameter('place_z_clearance', 0.05)
        self.declare_parameter('place_pose_timeout', 2.0)
        self.declare_parameter('place_lower_timeout', 20.0)
        self.declare_parameter(
            'filtered_cloud_topic', 'filtered_cloud')
        self.declare_parameter(
            'constrain_orientation', True)
        self.declare_parameter(
            'home_joints',
            [-0.0259, -0.4025, -0.0575, 2.0, 0.0604, 0.0722, 0.9141])

        self.base_link = self.get_parameter('base_link').value
        self.end_effector = self.get_parameter('end_effector_link').value
        self.gripper_joint = self.get_parameter('gripper_joint').value
        self.gripper_open = self.get_parameter('gripper_open').value
        self.gripper_closed = self.get_parameter('gripper_closed').value
        self.target_z_offset = self.get_parameter('target_z_offset').value
        self.require_target_box = self.get_parameter(
            'require_target_box').value
        self.box_wait_timeout = self.get_parameter(
            'box_wait_timeout').value
        self.force_threshold = self.get_parameter('force_threshold').value
        self.reach_tolerance = self.get_parameter('reach_tolerance').value
        self.reach_timeout = self.get_parameter('reach_timeout').value
        self.pre_approach_distance = self.get_parameter(
            'pre_approach_distance').value
        self.velocity_scaling = self.get_parameter('velocity_scaling').value
        self.idle_clear_interval = self.get_parameter(
            'idle_clear_interval').value
        self.insert_settle = self.get_parameter('insert_settle').value
        self.rebuild_timeout = self.get_parameter('rebuild_timeout').value
        self.box_apply_timeout = self.get_parameter(
            'box_apply_timeout').value
        self.box_remove_timeout = self.get_parameter(
            'box_remove_timeout').value
        self.place_pose_topic = self.get_parameter(
            'place_pose_topic').value
        self.place_cloud_topic = self.get_parameter(
            'place_cloud_topic').value
        self.place_filtered_cloud_topic = self.get_parameter(
            'place_filtered_cloud_topic').value
        self.place_z_clearance = self.get_parameter(
            'place_z_clearance').value
        self.place_pose_timeout = self.get_parameter(
            'place_pose_timeout').value
        self.place_lower_timeout = self.get_parameter(
            'place_lower_timeout').value
        self.home_joints = self.get_parameter('home_joints').value
        self.arm = MoveIt2(
            node=self,
            base_link=self.base_link,
            end_effector=self.end_effector,
            group_name=self.get_parameter('arm_group').value,
            constrain_orientation=self.get_parameter(
                'constrain_orientation').value,
        )
        self.gripper = GripperClient(
            self,
            joint_name=self.get_parameter('gripper_joint').value,
            open_width=self.gripper_open,
            closed_width=self.gripper_closed,
            force_threshold=self.force_threshold,
            width_tolerance=self.get_parameter(
                'gripper_width_tolerance').value,
            timeout=self.get_parameter('gripper_timeout').value,
        )
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.state = self.IDLE
        self.triggered = False
        self.validating_target = False
        self.target_pose = None
        self.target_hover_pose = None
        self._approach_phase = 0
        self.target_frame = None
        self.reach_wait_start = 0.0
        self.reach_ok_count = 0
        self.current_joints = [0.0] * 7
        self._prev_tick_joints = None
        self._joint_stable_ticks = 0
        self._last_joint_feedback = 0.0
        self._last_joint_delta = 0.0
        self._validation_sent = False
        self._stable_wait_logged = False
        self.home_start_time = 0.0
        self.stored_pose = None
        self.stored_frame = None
        self._latest_box = None
        self._latched_box = None
        self._pending_pose = None
        self._pending_time = 0.0
        self._clear_client = self.create_client(EmptySrv, 'clear_octomap')
        self._last_cloud_time = 0.0
        self._last_filtered_time = 0.0
        self._last_idle_clear = self.get_clock().now().nanoseconds * 1e-9
        self._clear_time = None
        self._clear_pending = False
        self._clear_success = False
        self._clear_done_time = 0.0
        self._clear_seq = 0
        self._rebuild_done = False
        self._t_trigger = None
        self._last_mark = None
        self._timing = {}
        self._box_applied_obj = None
        self._box_apply_start = 0.0
        self._box_remove_start = 0.0
        self._home_clear_waited = False
        self._home_clear_start = 0.0
        self.grasp_offset = 0.0
        self._latest_place_pose = None
        self._place_target_pose = None
        self._place_above_pose = None
        self._place_frame = None
        self._place_z = 0.0
        self._place_hover_z = 0.0
        self._place_validating = False
        self._place_clear_time = None
        self._place_rebuild_done = False
        self._place_validation_sent = False
        self._place_reach_ok_count = 0
        self._place_reach_start = 0.0
        self._place_cloud_time = 0.0
        self._place_filtered_time = 0.0
        self._place_cycle_home = False
        self._place_abort_open = False
        self.move_j_pub = self.create_publisher(
            JointState, 'control/move_j', 10)
        self.state_pub = self.create_publisher(
            Int32, 'grasp_executor_state', 10)
        self.map_update_pub = self.create_publisher(
            Bool, 'map_update_enable', 10)
        self.place_update_pub = self.create_publisher(
            Bool, 'place_update_enable', 10)

        self.create_subscription(
            PoseStamped, 'grasp_pose', self.grasp_callback, 10)
        self.create_subscription(
            Marker, 'yolo/target_box', self.target_box_callback, 10)
        self.create_subscription(
            PointCloud2, 'yolo/points_filtered', self.cloud_time_callback, 10)
        self.create_subscription(
            PointCloud2,
            self.get_parameter('filtered_cloud_topic').value,
            self.filtered_cloud_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.create_subscription(
            EmptyMsg, 'manual_grasp_start', self.manual_start_cb, 10)
        self.create_subscription(
            EmptyMsg, 'manual_release', self.manual_release_cb, 10)
        self.create_subscription(
            PoseStamped, self.place_pose_topic,
            self.place_pose_callback, 10)
        self.create_subscription(
            PointCloud2, self.place_cloud_topic,
            self.place_cloud_callback, 10)
        self.create_subscription(
            PointCloud2, self.place_filtered_cloud_topic,
            self.place_filtered_cloud_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        try:
            from agx_arm_msgs.msg import GripperStatus
            self.create_subscription(
                GripperStatus, 'feedback/gripper_status',
                self.gripper_feedback_cb, 10)
        except ImportError:
            self.get_logger().warning('GripperStatus import failed')
        self.create_subscription(
            JointState, 'feedback/joint_states',
            self.joint_feedback_cb, 10)
        self.create_timer(0.1, self.tick)
        self._init_timer = self.create_timer(0.5, self._init_open)
        self.get_logger().info('Nero seven-axis grasp executor ready')
        self.get_logger().info('[TIMING] phase timing instrumentation active')

    def target_box_callback(self, msg):
        box = {
            'id': 'grasp_target',
            'position': (
                msg.pose.position.x,
                msg.pose.position.y,
                msg.pose.position.z,
            ),
            'orientation': (
                msg.pose.orientation.x,
                msg.pose.orientation.y,
                msg.pose.orientation.z,
                msg.pose.orientation.w,
            ),
            'size': (
                msg.scale.x,
                msg.scale.y,
                msg.scale.z,
            ),
        }
        self._latest_box = (
            self.get_clock().now().nanoseconds * 1e-9, box)

    def cloud_time_callback(self, msg):
        del msg
        self._last_cloud_time = self.get_clock().now().nanoseconds * 1e-9

    def filtered_cloud_callback(self, msg):
        del msg
        self._last_filtered_time = self.get_clock().now().nanoseconds * 1e-9

    def place_pose_callback(self, msg):
        self._latest_place_pose = (
            self.get_clock().now().nanoseconds * 1e-9, msg)

    def place_cloud_callback(self, msg):
        del msg
        self._place_cloud_time = self.get_clock().now().nanoseconds * 1e-9

    def place_filtered_cloud_callback(self, msg):
        del msg
        self._place_filtered_time = self.get_clock().now().nanoseconds * 1e-9

    def grasp_callback(self, msg):
        if self.state != self.IDLE or self.validating_target:
            return
        now = self.get_clock().now().nanoseconds * 1e-9
        box = None
        if (self._latest_box is not None
                and now - self._latest_box[0] <= self.BOX_PAIR_WINDOW):
            box = self._latest_box[1]
        if self.require_target_box and box is None:
            self._pending_pose = msg
            self._pending_time = now
            return
        self._latched_box = box
        self._store_pose(msg.pose, msg.header.frame_id)

    def _store_pose(self, pose, frame_id):
        self.stored_pose = Pose(
            position=Point(
                x=pose.position.x,
                y=pose.position.y,
                z=pose.position.z + self.target_z_offset,
            ),
            orientation=Quaternion(
                x=pose.orientation.x,
                y=pose.orientation.y,
                z=pose.orientation.z,
                w=pose.orientation.w,
            ),
        )
        self.stored_frame = frame_id
        if self._latched_box is not None:
            box_bottom = (
                self._latched_box['position'][2]
                - self._latched_box['size'][2] / 2.0)
            self.grasp_offset = self.stored_pose.position.z - box_bottom
        else:
            self.grasp_offset = 0.0
        if not (-0.2 <= self.grasp_offset <= 0.3):
            self.get_logger().warning(
                f'Grasp offset {self.grasp_offset:.3f} out of range; '
                'clamping to 0.0')
            self.grasp_offset = 0.0
        self.get_logger().info(
            f'Target stored: ({pose.position.x:.3f}, '
            f'{pose.position.y:.3f}, '
            f'{pose.position.z:.3f}) '
            f'grasp_offset={self.grasp_offset:.3f}. '
            f'Waiting for /manual_grasp_start')

    def _hover_from_target(self, pose):
        q = pose.orientation
        rot = R.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
        z_axis = rot[:, 2]
        return Pose(
            position=Point(
                x=pose.position.x - z_axis[0] * self.pre_approach_distance,
                y=pose.position.y - z_axis[1] * self.pre_approach_distance,
                z=pose.position.z - z_axis[2] * self.pre_approach_distance,
            ),
            orientation=Quaternion(x=q.x, y=q.y, z=q.z, w=q.w),
        )

    def manual_start_cb(self, msg):
        del msg
        if self.state != self.IDLE or self.stored_pose is None:
            self.get_logger().warning(
                'Not ready for manual grasp start')
            return
        self.target_pose = self.stored_pose
        self.target_frame = self.stored_frame
        self.target_hover_pose = self._hover_from_target(self.target_pose)
        self._approach_phase = 0
        self.validating_target = True
        self._validation_sent = False
        self._stable_wait_logged = False
        self._rebuild_done = False
        self._clear_time = None
        self._box_applied = False
        self._box_apply_pending = False
        self._box_apply_done = False
        self._box_apply_ok = False
        self._box_remove_pending = False
        self._box_remove_done = False
        self._box_remove_ok = False
        self._place_validating = False
        self._place_rebuild_done = False
        self._place_validation_sent = False
        self._place_cycle_home = False
        self._place_abort_open = False
        p = self.target_pose.position
        self._t_trigger = self.get_clock().now().nanoseconds * 1e-9
        self._last_mark = self._t_trigger
        self._timing = {}
        self._mark('trigger')
        self.get_logger().info(
            f'Manual grasp triggered, validating: '
            f'({p.x:.3f}, {p.y:.3f}, {p.z:.3f})')

    def manual_release_cb(self, msg):
        del msg
        if self.state != self.WAIT_RELEASE or self._place_validating:
            return
        now = self.get_clock().now().nanoseconds * 1e-9
        if (self._latest_place_pose is None
                or now - self._latest_place_pose[0] > self.place_pose_timeout):
            self.get_logger().warning(
                'No fresh /place_pose available; opening gripper in place')
            self.state = self.OPEN_GRIPPER
            self.triggered = False
            return
        p = self._latest_place_pose[1].pose
        self._place_frame = self._latest_place_pose[1].header.frame_id
        self._place_z = p.position.z + self.grasp_offset
        self._place_hover_z = self._place_z + self.place_z_clearance
        ori = self.stored_pose.orientation if self.stored_pose is not None \
            else Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        self._place_target_pose = Pose(
            position=Point(x=p.position.x, y=p.position.y, z=self._place_z),
            orientation=Quaternion(
                x=ori.x, y=ori.y, z=ori.z, w=ori.w),
        )
        self._place_above_pose = Pose(
            position=Point(
                x=p.position.x, y=p.position.y, z=self._place_hover_z),
            orientation=Quaternion(
                x=ori.x, y=ori.y, z=ori.z, w=ori.w),
        )
        self._place_clear_time = None
        self._place_rebuild_done = False
        self._place_validation_sent = False
        self._place_reach_ok_count = 0
        self._stable_wait_logged = False
        self._place_validating = True
        self.get_logger().info(
            f'Manual place triggered: table_z={p.position.z:.3f} '
            f'grasp_offset={self.grasp_offset:.3f} '
            f'place_z={self._place_z:.3f}')

    def gripper_feedback_cb(self, msg):
        self.gripper.feedback(msg.width, msg.force)

    def joint_feedback_cb(self, msg):
        names = ['joint1', 'joint2', 'joint3', 'joint4',
                 'joint5', 'joint6', 'joint7']
        for n, name in enumerate(names):
            if name in msg.name:
                idx = msg.name.index(name)
                if idx < len(msg.position):
                    self.current_joints[n] = msg.position[idx]
        self._last_joint_feedback = self.get_clock().now().nanoseconds * 1e-9

    def tick(self):
        self.state_pub.publish(Int32(data=self.state))
        self.gripper.update(0.1)
        map_update = Bool()
        map_update.data = (self.state == self.IDLE)
        self.map_update_pub.publish(map_update)
        place_update = Bool()
        place_update.data = (
            self.state == self.WAIT_RELEASE
            and self._place_validating
            and not self._place_rebuild_done
            and not self._place_validation_sent)
        self.place_update_pub.publish(place_update)

        if self.state == self.IDLE and self.validating_target:
            if not self._box_applied:
                if not self._box_apply_pending:
                    if self._latched_box is None:
                        self.get_logger().error(
                            'No target box available; aborting validation')
                        self.validating_target = False
                        return
                    self._box_apply_pending = True
                    self._box_apply_done = False
                    self._box_apply_ok = False
                    self._box_apply_start = (
                        self.get_clock().now().nanoseconds * 1e-9)
                    if not self.arm.apply_collision_object(
                            self._latched_box, add=True,
                            callback=self._box_apply_cb):
                        self.get_logger().error(
                            'Cannot apply target box; aborting validation')
                        self.validating_target = False
                        return
                    self.get_logger().info(
                        'Applying target box to planning scene...')
                    return
                if not self._box_apply_done:
                    now_apply = self.get_clock().now().nanoseconds * 1e-9
                    if (now_apply - self._box_apply_start
                            > self.box_apply_timeout):
                        self.get_logger().error(
                            'Target box apply timed out; aborting validation')
                        self.validating_target = False
                        self._box_apply_pending = False
                    return
                if not self._box_apply_ok:
                    self.get_logger().error(
                        'Target box apply failed; aborting validation')
                    self.validating_target = False
                    return
                self._box_applied = True
                self._box_applied_obj = self._latched_box
                self._mark('box_applied')
            if not self._rebuild_done:
                now_clear = self.get_clock().now().nanoseconds * 1e-9
                if self._clear_time is None:
                    self._clear_time = now_clear
                    self._call_clear()
                    self._mark('clear_sent')
                    return
                if self._clear_pending:
                    if now_clear - self._clear_time > self.rebuild_timeout:
                        self.get_logger().warning(
                            'Clear confirmation timed out; '
                            'using fallback rebuild detection')
                        self._clear_pending = False
                        self._clear_success = False
                    else:
                        return
                if self._clear_success and self._last_filtered_time > 0.0:
                    if (self._last_filtered_time > self._clear_done_time
                            and now_clear - self._last_filtered_time
                            >= self.insert_settle):
                        self._set_rebuild_done()
                    elif (now_clear - self._clear_done_time
                            > self.rebuild_timeout):
                        self.get_logger().warning(
                            'Timed out waiting for octomap rebuild; '
                            'planning anyway')
                        self._set_rebuild_done()
                    else:
                        return
                else:
                    if now_clear - self._clear_time > self.rebuild_timeout:
                        self.get_logger().warning(
                            'Timed out waiting for octomap rebuild; '
                            'planning anyway')
                        self._set_rebuild_done()
                    elif (self._last_cloud_time > self._clear_time
                            and now_clear - self._last_cloud_time
                            >= self.insert_settle):
                        self._set_rebuild_done()
                    else:
                        return
            if not self._validation_sent:
                if not self._joints_stable():
                    self._log_stable_wait()
                    return
                self._mark('validate_sent')
                self.arm.move_to_pose(
                    self.target_pose, self.target_frame, plan_only=True)
                self._validation_sent = True
                return
            if not self.arm.is_done():
                return
            self.validating_target = False
            if self.arm.success:
                self._mark('validate_done')
                self.state = self.MOVE_TO_TARGET
                self.get_logger().info(
                    'Target is reachable; starting grasp sequence')
            else:
                self._mark('validate_failed')
                self.target_pose = None
                self.target_frame = None
                self._fire_box_remove()
                self.get_logger().warning(
                    'Target is unreachable; remaining IDLE')
            return

        if self.state == self.IDLE:
            if self._pending_pose is not None:
                now_pending = self.get_clock().now().nanoseconds * 1e-9
                if (self._latest_box is not None
                        and now_pending - self._latest_box[0]
                        <= self.BOX_PAIR_WINDOW):
                    self._latched_box = self._latest_box[1]
                    self._store_pose(
                        self._pending_pose.pose,
                        self._pending_pose.header.frame_id)
                    self._pending_pose = None
                elif (now_pending - self._pending_time
                        > self.box_wait_timeout):
                    self.get_logger().warning(
                        'No paired target box; grasp rejected')
                    self._pending_pose = None
            if self.idle_clear_interval > 0:
                now_idle = self.get_clock().now().nanoseconds * 1e-9
                if now_idle - self._last_idle_clear >= self.idle_clear_interval:
                    self._last_idle_clear = now_idle
                    self._call_clear(verbose=False)
            return
        if self.state == self.OPEN_GRIPPER:
            if not self.triggered:
                self.gripper.open()
                self.triggered = True
            elif self.gripper.done:
                self.state = self.IDLE
                self.triggered = False
            return
        if self.state == self.MOVE_TO_TARGET:
            if not self.triggered:
                if not self._joints_stable():
                    self._log_stable_wait()
                    return
                self._approach_phase = 0
                self._mark('plan_sent')
                self.arm.move_to_pose(
                    self.target_hover_pose, self.target_frame,
                    velocity_scaling=self.velocity_scaling)
                self.triggered = True
            elif self.arm.is_done():
                if not self.arm.success:
                    self._mark('plan_failed')
                    self.get_logger().error('Arm motion to hover failed')
                    self._enter_home()
                    return
                if self._approach_phase == 0:
                    self._approach_phase = 1
                    self._mark('cartesian_sent')
                    self.arm.move_cartesian_to(
                        [self.target_pose], self.target_frame)
                elif self._approach_phase == 1:
                    if self.arm.success:
                        self._mark('plan_done')
                        self.state = self.WAIT_REACH
                        self.reach_wait_start = (
                            self.get_clock().now().nanoseconds * 1e-9)
                        self.triggered = False
                    else:
                        self._mark('plan_failed')
                        self.get_logger().error(
                            'Cartesian descent failed; going home')
                        self._enter_home()
            return
        if self.state == self.WAIT_REACH:
            elapsed = (
                self.get_clock().now().nanoseconds * 1e-9
                - self.reach_wait_start)
            if elapsed < 1.0:
                self.reach_ok_count = 0
                return
            error = self._tcp_error()
            if error is not None and error <= self.reach_tolerance:
                self.reach_ok_count += 1
            else:
                self.reach_ok_count = 0
            if self.reach_ok_count >= 3:
                self._mark('reached')
                self.get_logger().info(
                    f'TCP reached target (error={error:.4f} m)')
                self.state = self.CLOSE_GRIPPER
                return
            if elapsed > self.reach_timeout:
                self.get_logger().warning(
                    f'TCP reach timeout ({self.reach_timeout:.1f}s), '
                    f'error={error}')
                self._enter_home()
            return
        if self.state == self.CLOSE_GRIPPER:
            if not self.triggered:
                self.gripper.close()
                self.triggered = True
            elif self.gripper.done:
                self._mark('gripper_done')
                if self.gripper.holding():
                    self.get_logger().info(
                        'Grasp sequence completed (object held)')
                    self._latched_box = None
                else:
                    self.get_logger().info(
                        'Grasp sequence completed (nothing grasped)')
                self._enter_home()
            return
        if self.state == self.MOVE_HOME:
            if not self.triggered:
                if not self._ensure_box_removed():
                    return
                if not self._joints_stable():
                    self._log_stable_wait()
                    return
                if not self._home_clear_waited:
                    self._home_clear_waited = True
                    self._home_clear_start = (
                        self.get_clock().now().nanoseconds * 1e-9)
                    self._call_clear(verbose=False)
                    self.get_logger().info(
                        'Clearing octomap before home move')
                    return
                if self._clear_pending:
                    now_home = self.get_clock().now().nanoseconds * 1e-9
                    if (now_home - self._home_clear_start > 5.0):
                        self.get_logger().warning(
                            'Home octomap clear timeout; proceeding')
                        self._clear_pending = False
                    else:
                        return
                self._mark('home_sent')
                self.arm.move_to_joints(
                    self.home_joints,
                    velocity_scaling=self.velocity_scaling)
                self.home_start_time = (
                    self.get_clock().now().nanoseconds * 1e-9)
                self.get_logger().info('Moving to home position')
                self.triggered = True
            elif self.arm.is_done():
                if self.arm.success:
                    home = self.home_joints
                    errors = [abs(self.current_joints[i] - home[i])
                              for i in range(7)]
                    if max(errors) < 0.05:
                        self._mark('home_done')
                        self.get_logger().info(
                            'Reached home position')
                        if self._place_abort_open:
                            self._place_abort_open = False
                            self.get_logger().info(
                                'Place failed; opening gripper at home')
                            self.state = self.OPEN_GRIPPER
                        elif self._place_cycle_home:
                            self._place_cycle_home = False
                            self.state = self.IDLE
                        else:
                            self.state = self.WAIT_RELEASE
                        self.triggered = False
                        self._cycle_summary()
                    else:
                        elapsed = (self.get_clock().now().nanoseconds * 1e-9
                                   - self.home_start_time)
                        if elapsed > 90.0:
                            self._mark('home_timeout')
                            self.get_logger().warning(
                                'Home move timeout, '
                                f'max error={max(errors):.3f} rad')
                            self.state = self.WAIT_RELEASE
                            self.triggered = False
                            self._cycle_summary()
                else:
                    self._mark('home_plan_failed')
                    self.get_logger().error('Home move failed')
                    self.state = self.IDLE
                    self.triggered = False
                    self._cycle_summary()
            return
        if self.state == self.WAIT_RELEASE:
            if self._place_validating:
                if not self._place_rebuild_done:
                    now_place = (
                        self.get_clock().now().nanoseconds * 1e-9)
                    if self._place_clear_time is None:
                        self._place_clear_time = now_place
                        self._call_clear()
                        self._mark('place_clear_sent')
                        return
                    if self._clear_pending:
                        if (now_place - self._place_clear_time
                                > self.rebuild_timeout):
                            self.get_logger().warning(
                                'Place clear confirmation timed out; '
                                'using fallback rebuild detection')
                            self._clear_pending = False
                            self._clear_success = False
                        else:
                            return
                    if (self._place_filtered_time > self._place_clear_time
                            and now_place - self._place_filtered_time
                            >= self.insert_settle):
                        self._place_rebuild_done = True
                        self._mark('place_rebuild_done')
                    elif (now_place - self._place_clear_time
                            > self.rebuild_timeout):
                        self.get_logger().warning(
                            'Place octomap rebuild timeout; planning anyway')
                        self._place_rebuild_done = True
                    else:
                        return
                if not self._place_validation_sent:
                    if not self._joints_stable():
                        self._log_stable_wait()
                        return
                    self._place_validation_sent = True
                    self._mark('place_validate_sent')
                    self.arm.move_to_pose(
                        self._place_target_pose, self._place_frame,
                        plan_only=True)
                    return
                if not self.arm.is_done():
                    return
                self._place_validating = False
                if self.arm.success:
                    self._mark('place_validate_done')
                    self.get_logger().info(
                        'Place point reachable; starting place sequence')
                    self.state = self.MOVE_TO_PLACE_ABOVE
                    self.triggered = False
                else:
                    self._mark('place_validate_failed')
                    self.get_logger().warning(
                        'Place point unreachable; opening gripper in place')
                    self.state = self.OPEN_GRIPPER
                    self.triggered = False
            return
        if self.state == self.MOVE_TO_PLACE_ABOVE:
            if not self.triggered:
                self._mark('place_above_sent')
                self.arm.move_to_pose(
                    self._place_above_pose, self._place_frame,
                    velocity_scaling=self.velocity_scaling)
                self.triggered = True
            elif self.arm.is_done():
                if self.arm.success:
                    self.state = self.LOWER_TO_PLACE
                    self.triggered = False
                    self._place_reach_ok_count = 0
                else:
                    self.get_logger().error('Move to place-above failed')
                    self._place_abort_open = True
                    self._enter_home()
            return
        if self.state == self.LOWER_TO_PLACE:
            if not self.triggered:
                if not self._joints_stable():
                    self._log_stable_wait()
                    return
                self._mark('place_lower_sent')
                self.arm.move_to_pose(
                    self._place_target_pose, self._place_frame,
                    velocity_scaling=self.velocity_scaling,
                    constrain_orientation=True,
                    orientation_tolerance=0.35)
                self.triggered = True
                self._place_reach_start = (
                    self.get_clock().now().nanoseconds * 1e-9)
            elif self.arm.is_done():
                if not self.arm.success:
                    self.get_logger().error('Lower to place failed')
                    self._place_abort_open = True
                    self._enter_home()
                    return
                elapsed = (
                    self.get_clock().now().nanoseconds * 1e-9
                    - self._place_reach_start)
                if elapsed < 1.0:
                    self._place_reach_ok_count = 0
                    return
                error = self._tcp_error_to(
                    self._place_target_pose, self._place_frame)
                if error is not None and error <= self.reach_tolerance:
                    self._place_reach_ok_count += 1
                else:
                    self._place_reach_ok_count = 0
                if self._place_reach_ok_count >= 3:
                    self._mark('place_reached')
                    self.state = self.PLACE_OPEN
                    self.triggered = False
                elif elapsed > self.place_lower_timeout:
                    self.get_logger().warning(
                        f'Place reach timeout, error={error}; '
                        'releasing in place')
                    self.state = self.PLACE_OPEN
                    self.triggered = False
            return
        if self.state == self.PLACE_OPEN:
            if not self.triggered:
                self.gripper.open()
                self.triggered = True
            elif self.gripper.done:
                self._mark('place_opened')
                self.get_logger().info('Object released at place spot')
                self.state = self.PLACE_LIFT
                self.triggered = False
            return
        if self.state == self.PLACE_LIFT:
            if not self.triggered:
                self.arm.move_to_pose(
                    self._place_above_pose, self._place_frame,
                    velocity_scaling=self.velocity_scaling)
                self.triggered = True
            elif self.arm.is_done():
                if self.arm.success:
                    self.get_logger().info('Lifted clear of placed object')
                    self._place_cycle_home = True
                else:
                    self.get_logger().error('Lift after place failed')
                self._enter_home()
            return

    def _enter_home(self):
        self.state = self.MOVE_HOME
        self.triggered = False
        self._home_clear_waited = False
        self._home_clear_start = 0.0

    def _init_open(self):
        self.state = self.OPEN_GRIPPER
        self.triggered = False
        self._init_timer.cancel()

    def _call_clear(self, verbose=True):
        if not self._clear_client.service_is_ready():
            self.get_logger().warning('/clear_octomap service not ready')
            return False
        self._clear_pending = True
        self._clear_success = False
        self._clear_seq += 1
        seq = self._clear_seq
        future = self._clear_client.call_async(EmptySrv.Request())
        future.add_done_callback(
            lambda fut, s=seq, v=verbose: self._clear_done_cb(fut, s, v))
        return True

    def _clear_done_cb(self, future, seq, verbose=True):
        if seq != self._clear_seq:
            return
        self._clear_pending = False
        self._clear_done_time = self.get_clock().now().nanoseconds * 1e-9
        try:
            future.result()
        except Exception:
            self._clear_success = False
            self.get_logger().error('Octomap clear failed')
            return
        self._clear_success = True
        if self.validating_target:
            self._mark('clear_done')
        if verbose:
            self.get_logger().info('Octomap cleared (confirmed)')

    def _box_apply_cb(self, future):
        self._box_apply_done = True
        try:
            self._box_apply_ok = bool(future.result().success)
        except Exception:
            self._box_apply_ok = False
        if self._box_apply_ok:
            self.get_logger().info('Target box applied (confirmed)')
        else:
            self.get_logger().error('Target box apply failed')

    def _box_remove_cb(self, future):
        self._box_remove_done = True
        self._box_remove_pending = False
        try:
            self._box_remove_ok = bool(future.result().success)
        except Exception:
            self._box_remove_ok = False
        if self._box_remove_ok:
            self._box_applied = False
            self.get_logger().info('Target box removed (confirmed)')
        else:
            self.get_logger().error('Target box remove failed')

    def _ensure_box_removed(self):
        if not self._box_applied or self._box_remove_done:
            return True
        if not self._box_remove_pending:
            self._box_remove_pending = True
            self._box_remove_start = (
                self.get_clock().now().nanoseconds * 1e-9)
            obj = self._box_applied_obj
            if obj is None:
                self._box_applied = False
                return True
            if not self.arm.apply_collision_object(
                    obj, add=False, callback=self._box_remove_cb):
                self.get_logger().error(
                    'Cannot remove target box; proceeding anyway')
                self._box_applied = False
                return True
            self.get_logger().info(
                'Removing target box from planning scene...')
            return False
        now_remove = self.get_clock().now().nanoseconds * 1e-9
        if (now_remove - self._box_remove_start
                > self.box_remove_timeout):
            self.get_logger().warning(
                'Target box remove timed out; proceeding anyway')
            self._box_remove_done = True
            self._box_applied = False
            return True
        return False

    def _fire_box_remove(self):
        if not self._box_applied:
            return
        self._box_applied = False
        obj = self._box_applied_obj
        self._box_applied_obj = None
        if obj is None:
            return
        self.arm.apply_collision_object(obj, add=False, callback=None)
        self.get_logger().info('Target box remove requested')

    def _mark(self, name):
        if self._t_trigger is None:
            return
        now = self.get_clock().now().nanoseconds * 1e-9
        self._timing[name] = now
        since_prev = (now - self._last_mark) if self._last_mark is not None else 0.0
        since_start = now - self._t_trigger
        self._last_mark = now
        self.get_logger().info(
            f'[TIMING] {name}: +{since_prev:.2f}s (cum {since_start:.2f}s)')

    def _set_rebuild_done(self):
        if self._rebuild_done:
            return
        self._rebuild_done = True
        self._mark('rebuild_done')

    def _cycle_summary(self):
        t = self._timing
        if 'trigger' not in t:
            return
        rows = [
            ('clear_sent', 'trigger', 'pre-clear'),
            ('clear_done', 'clear_sent', 'clear_srv'),
            ('rebuild_done', 'clear_done', 'rebuild'),
            ('validate_sent', 'rebuild_done', 'stable_wait'),
            ('validate_done', 'validate_sent', 'validate_plan'),
            ('plan_sent', 'validate_done', 'between_plans'),
            ('plan_done', 'plan_sent', 'exec_plan'),
            ('reached', 'plan_done', 'traj_exec'),
            ('gripper_done', 'reached', 'gripper_close'),
            ('home_done', 'gripper_done', 'home_move'),
        ]
        parts = []
        for later, earlier, label in rows:
            if later in t and earlier in t:
                parts.append(f'{label}={t[later] - t[earlier]:.2f}s')
        tail = ''
        if 'home_done' in t:
            tail = f'  total={t["home_done"] - t["trigger"]:.2f}s'
        self.get_logger().info('[TIMING] breakdown: ' + ' '.join(parts) + tail)

    def _joints_stable(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        if now - self._last_joint_feedback > 0.5:
            self._joint_stable_ticks = 0
            self._last_joint_delta = -1.0
            return False
        if self._prev_tick_joints is None:
            self._prev_tick_joints = list(self.current_joints)
            return False
        delta = max(abs(a - b) for a, b in zip(
            self.current_joints, self._prev_tick_joints))
        self._prev_tick_joints = list(self.current_joints)
        self._last_joint_delta = delta
        if delta < 0.01:
            self._joint_stable_ticks += 1
        else:
            self._joint_stable_ticks = 0
        return self._joint_stable_ticks >= 3

    def _log_stable_wait(self):
        if self._stable_wait_logged:
            return
        self._stable_wait_logged = True
        self.get_logger().warning(
            'Arm joints not stable; waiting for settle before planning '
            f'(max_delta={self._last_joint_delta:.4f} rad)')

    def _tcp_error_to(self, target_pose, target_frame):
        try:
            transform = self.tf_buffer.lookup_transform(
                target_frame,
                self.end_effector,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=1.0),
            )
        except TransformException:
            return None
        actual = transform.transform.translation
        target = target_pose.position
        return ((actual.x - target.x) ** 2 +
                (actual.y - target.y) ** 2 +
                (actual.z - target.z) ** 2) ** 0.5

    def _tcp_error(self):
        return self._tcp_error_to(self.target_pose, self.target_frame)

    def report_position_error(self):
        try:
            transform = self.tf_buffer.lookup_transform(
                self.target_frame,
                self.end_effector,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=1.0),
            )
        except TransformException as exc:
            self.get_logger().warn(f'TCP comparison unavailable: {exc}')
            return
        actual = transform.transform.translation
        target = self.target_pose.position
        error = ((actual.x - target.x) ** 2 +
                 (actual.y - target.y) ** 2 +
                 (actual.z - target.z) ** 2) ** 0.5
        self.get_logger().info(f'TCP position error: {error:.4f} m')


def main():
    rclpy.init()
    node = GraspExecutor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
