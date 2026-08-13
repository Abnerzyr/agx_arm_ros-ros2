#!/usr/bin/env python3

import rclpy
from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Empty as EmptyMsg
from tf2_ros import Buffer, TransformException, TransformListener

from agx_arm_vision.moveit2_local import MoveIt2


class GraspExecutor(Node):
    IDLE = 0
    OPEN_GRIPPER = 1
    MOVE_TO_TARGET = 2
    WAIT_REACH = 3
    CLOSE_GRIPPER = 4
    MOVE_HOME = 5
    WAIT_RELEASE = 6

    def __init__(self):
        super().__init__('grasp_executor')
        self.declare_parameter('base_link', 'base_link')
        self.declare_parameter('end_effector_link', 'tcp_link')
        self.declare_parameter('arm_group', 'arm')
        self.declare_parameter(
            'gripper_action', '/gripper_controller/follow_joint_trajectory')
        self.declare_parameter('gripper_joint', 'gripper')
        self.declare_parameter('gripper_open', 0.1)
        self.declare_parameter('gripper_closed', 0.0)
        self.declare_parameter('target_z_offset', 0.0)
        self.declare_parameter('rejected_target_distance', 0.1)
        self.declare_parameter('force_threshold', 1.5)
        self.declare_parameter('reach_tolerance', 0.03)
        self.declare_parameter('reach_timeout', 10.0)
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
        self.rejected_target_distance = self.get_parameter(
            'rejected_target_distance').value
        self.force_threshold = self.get_parameter('force_threshold').value
        self.reach_tolerance = self.get_parameter('reach_tolerance').value
        self.reach_timeout = self.get_parameter('reach_timeout').value
        self.home_joints = self.get_parameter('home_joints').value
        self.arm = MoveIt2(
            node=self,
            base_link=self.base_link,
            end_effector=self.end_effector,
            group_name=self.get_parameter('arm_group').value,
            constrain_orientation=self.get_parameter(
                'constrain_orientation').value,
        )
        self.gripper_pub = self.create_publisher(
            JointState, '/control/gripper_target', 1)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.state = self.IDLE
        self.triggered = False
        self.validating_target = False
        self.gripper_done = True
        self.target_pose = None
        self.target_frame = None
        self.rejected_target = None
        self.current_force = 0.0
        self.reach_wait_start = 0.0
        self.reach_ok_count = 0
        self.current_joints = [0.0] * 7
        self.home_start_time = 0.0
        self.stored_pose = None
        self.stored_frame = None
        self.move_j_pub = self.create_publisher(
            JointState, '/control/move_j', 10)

        self.create_subscription(
            PoseStamped, '/grasp_pose', self.grasp_callback, 10)
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

    def grasp_callback(self, msg):
        if self.state != self.IDLE:
            return
        self.stored_pose = Pose(
            position=Point(
                x=msg.pose.position.x,
                y=msg.pose.position.y,
                z=msg.pose.position.z + self.target_z_offset,
            ),
            orientation=Quaternion(
                x=msg.pose.orientation.x,
                y=msg.pose.orientation.y,
                z=msg.pose.orientation.z,
                w=msg.pose.orientation.w,
            ),
        )
        self.stored_frame = msg.header.frame_id
        self.get_logger().info(
            f'Target stored: ({msg.pose.position.x:.3f}, '
            f'{msg.pose.position.y:.3f}, '
            f'{msg.pose.position.z:.3f}). '
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
        self.arm.move_to_pose(
            self.target_pose, self.target_frame, plan_only=True)
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

    def send_gripper(self, position):
        self.gripper_done = False
        msg = JointState()
        msg.name = [self.gripper_joint]
        msg.position = [float(position)]
        self.gripper_pub.publish(msg)
        self._gripper_timer = self.create_timer(1.5, self._on_gripper_done)

    def _on_gripper_done(self):
        self.gripper_done = True
        self._gripper_timer.cancel()

    def gripper_feedback_cb(self, msg):
        self.current_force = msg.force
        if (self.state == self.CLOSE_GRIPPER
                and self.triggered
                and msg.force > self.force_threshold):
            self.get_logger().info(
                f'Object grasped! force={msg.force:.2f} N, '
                f'width={msg.width:.3f} m')
            self.gripper_done = True

    def joint_feedback_cb(self, msg):
        names = ['joint1', 'joint2', 'joint3', 'joint4',
                 'joint5', 'joint6', 'joint7']
        for n, name in enumerate(names):
            if name in msg.name:
                idx = msg.name.index(name)
                if idx < len(msg.position):
                    self.current_joints[n] = msg.position[idx]

    def tick(self):
        if self.state == self.IDLE and self.validating_target:
            if not self.arm.is_done():
                return
            self.validating_target = False
            if self.arm.success:
                self.state = self.MOVE_TO_TARGET
                self.get_logger().info(
                    'Target is reachable; starting grasp sequence')
            else:
                p = self.target_pose.position
                self.rejected_target = (
                    self.target_frame, p.x, p.y, p.z)
                self.target_pose = None
                self.target_frame = None
                self.get_logger().warning(
                    'Target is unreachable; remaining IDLE')
            return

        if self.state == self.IDLE:
            return
        if self.state == self.OPEN_GRIPPER:
            if not self.triggered:
                self.send_gripper(self.gripper_open)
                self.triggered = True
            elif self.gripper_done:
                self.state = self.IDLE
                self.triggered = False
            return
        if self.state == self.MOVE_TO_TARGET:
            if not self.triggered:
                self.arm.move_to_pose(self.target_pose, self.target_frame)
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
                self.current_force = 0.0
                self.send_gripper(self.gripper_closed)
                self.triggered = True
            elif self.gripper_done:
                if not hasattr(self, '_home_wait'):
                    self._home_wait = 0
                if self._home_wait < 5:
                    self._home_wait += 1
                    return
                self._home_wait = 0
                if self.current_force > self.force_threshold:
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
                self.arm.move_to_joints(self.home_joints)
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
                        if elapsed > 15.0:
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
