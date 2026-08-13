#!/usr/bin/env python3

import rclpy
from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion
from rclpy.node import Node
from sensor_msgs.msg import JointState, PointCloud2
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

    TARGET_ALLOWED_LINKS = ('tcp_link', 'gripper_base')
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
        self.declare_parameter('velocity_scaling', 0.1)
        self.declare_parameter('idle_clear_interval', 10.0)
        self.declare_parameter('insert_settle', 0.3)
        self.declare_parameter('rebuild_timeout', 2.5)
        self.declare_parameter(
            'constrain_orientation', True)
        self.declare_parameter(
            'home_joints',
            [-1.751, -0.342, 1.656, 1.036, 0.360, 0.074, 1.570])

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
        self.velocity_scaling = self.get_parameter('velocity_scaling').value
        self.idle_clear_interval = self.get_parameter(
            'idle_clear_interval').value
        self.insert_settle = self.get_parameter('insert_settle').value
        self.rebuild_timeout = self.get_parameter('rebuild_timeout').value
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
        self._clear_client = self.create_client(EmptySrv, '/clear_octomap')
        self._last_cloud_time = 0.0
        self._last_idle_clear = self.get_clock().now().nanoseconds * 1e-9
        self._clear_time = None
        self._rebuild_done = False
        self.move_j_pub = self.create_publisher(
            JointState, '/control/move_j', 10)

        self.create_subscription(
            PoseStamped, '/grasp_pose', self.grasp_callback, 10)
        self.create_subscription(
            Marker, '/yolo/target_box', self.target_box_callback, 10)
        self.create_subscription(
            PointCloud2, '/yolo/points_filtered', self.cloud_time_callback, 10)
        self.create_subscription(
            EmptyMsg, '/manual_grasp_start', self.manual_start_cb, 10)
        self.create_subscription(
            EmptyMsg, '/manual_release', self.manual_release_cb, 10)
        try:
            from agx_arm_msgs.msg import GripperStatus
            self.create_subscription(
                GripperStatus, '/feedback/gripper_status',
                self.gripper_feedback_cb, 10)
        except ImportError:
            self.get_logger().warning('GripperStatus import failed')
        self.create_subscription(
            JointState, '/feedback/joint_states',
            self.joint_feedback_cb, 10)
        self.create_timer(0.1, self.tick)
        self._init_timer = self.create_timer(0.5, self._init_open)
        self.get_logger().info('Nero seven-axis grasp executor ready')

    def target_box_callback(self, msg):
        box = {
            'id': 'grasp_target',
            'position': (
                msg.pose.position.x,
                msg.pose.position.y,
                msg.pose.position.z,
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

    def grasp_callback(self, msg):
        if self.state != self.IDLE:
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
        self.get_logger().info(
            f'Target stored: ({pose.position.x:.3f}, '
            f'{pose.position.y:.3f}, '
            f'{pose.position.z:.3f}). '
            f'Waiting for /manual_grasp_start')

    def manual_start_cb(self, msg):
        del msg
        if self.state != self.IDLE or self.stored_pose is None:
            self.get_logger().warning(
                'Not ready for manual grasp start')
            return
        self.target_pose = self.stored_pose
        self.target_frame = self.stored_frame
        self.validating_target = True
        self._validation_sent = False
        self._stable_wait_logged = False
        self._rebuild_done = False
        self._clear_time = None
        p = self.target_pose.position
        self.get_logger().info(
            f'Manual grasp triggered, validating: '
            f'({p.x:.3f}, {p.y:.3f}, {p.z:.3f})')

    def manual_release_cb(self, msg):
        del msg
        if self.state != self.WAIT_RELEASE:
            return
        self.get_logger().info('Manual release triggered')
        self.state = self.OPEN_GRIPPER
        self.triggered = False

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
        self.gripper.update(0.1)

        if self.state == self.IDLE and self.validating_target:
            if not self._rebuild_done:
                now_clear = self.get_clock().now().nanoseconds * 1e-9
                if self._clear_time is None:
                    self._clear_time = now_clear
                    self._call_clear()
                    return
                if now_clear - self._clear_time > self.rebuild_timeout:
                    self.get_logger().warning(
                        'Timed out waiting for octomap rebuild; '
                        'planning anyway')
                    self._rebuild_done = True
                elif (self._last_cloud_time > self._clear_time
                        and now_clear - self._last_cloud_time
                        >= self.insert_settle):
                    self._rebuild_done = True
                else:
                    return
            if not self._validation_sent:
                if not self._joints_stable():
                    self._log_stable_wait()
                    return
                self.arm.move_to_pose(
                    self.target_pose, self.target_frame, plan_only=True,
                    collision_objects=self._latched_box,
                    allowed_links=self.TARGET_ALLOWED_LINKS)
                self._validation_sent = True
                return
            if not self.arm.is_done():
                return
            self.validating_target = False
            if self.arm.success:
                self.state = self.MOVE_TO_TARGET
                self.get_logger().info(
                    'Target is reachable; starting grasp sequence')
            else:
                self.target_pose = None
                self.target_frame = None
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
                    self._call_clear()
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
                self.arm.move_to_pose(
                    self.target_pose, self.target_frame,
                    velocity_scaling=self.velocity_scaling,
                    collision_objects=self._latched_box,
                    allowed_links=self.TARGET_ALLOWED_LINKS)
                self.triggered = True
            elif self.arm.is_done():
                if self.arm.success:
                    self.state = self.WAIT_REACH
                    self.reach_wait_start = self.get_clock().now().nanoseconds * 1e-9
                else:
                    self.get_logger().error('Arm motion failed')
                    self.state = self.MOVE_HOME
                self.triggered = False
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
                self.get_logger().info(
                    f'TCP reached target (error={error:.4f} m)')
                self.state = self.CLOSE_GRIPPER
                return
            if elapsed > self.reach_timeout:
                self.get_logger().warning(
                    f'TCP reach timeout ({self.reach_timeout:.1f}s), '
                    f'error={error}')
                self.state = self.MOVE_HOME
            return
        if self.state == self.CLOSE_GRIPPER:
            if not self.triggered:
                self.gripper.close()
                self.triggered = True
            elif self.gripper.done:
                if self.gripper.holding():
                    self.get_logger().info(
                        'Grasp sequence completed (object held)')
                else:
                    self.get_logger().info(
                        'Grasp sequence completed (nothing grasped)')
                self.state = self.MOVE_HOME
                self.triggered = False
            return
        if self.state == self.MOVE_HOME:
            if not self.triggered:
                if not self._joints_stable():
                    self._log_stable_wait()
                    return
                self.arm.move_to_joints(
                    self.home_joints,
                    velocity_scaling=self.velocity_scaling,
                    collision_objects=self._latched_box,
                    allowed_links=self.TARGET_ALLOWED_LINKS)
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
                        self.get_logger().info(
                            'Reached home position')
                        self.state = self.WAIT_RELEASE
                        self.triggered = False
                    else:
                        elapsed = (self.get_clock().now().nanoseconds * 1e-9
                                   - self.home_start_time)
                        if elapsed > 90.0:
                            self.get_logger().warning(
                                'Home move timeout, '
                                f'max error={max(errors):.3f} rad')
                            self.state = self.WAIT_RELEASE
                            self.triggered = False
                else:
                    self.get_logger().error('Home move failed')
                    self.state = self.IDLE
                    self.triggered = False
            return
        if self.state == self.WAIT_RELEASE:
            return

    def _init_open(self):
        self.state = self.OPEN_GRIPPER
        self.triggered = False
        self._init_timer.cancel()

    def _call_clear(self):
        if not self._clear_client.service_is_ready():
            self.get_logger().warning('/clear_octomap service not ready')
            return
        self._clear_client.call_async(EmptySrv.Request())
        self.get_logger().info('Octomap cleared')

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

    def _tcp_error(self):
        try:
            transform = self.tf_buffer.lookup_transform(
                self.target_frame,
                self.end_effector,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=1.0),
            )
        except TransformException:
            return None
        actual = transform.transform.translation
        target = self.target_pose.position
        return ((actual.x - target.x) ** 2 +
                (actual.y - target.y) ** 2 +
                (actual.z - target.z) ** 2) ** 0.5

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
