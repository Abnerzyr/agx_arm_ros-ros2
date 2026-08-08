#!/usr/bin/env python3

import rclpy
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion
from rclpy.action import ActionClient
from rclpy.node import Node
from tf2_ros import Buffer, TransformException, TransformListener
from trajectory_msgs.msg import JointTrajectoryPoint

from agx_arm_vision.moveit2_local import MoveIt2


class GraspExecutor(Node):
    IDLE = 0
    OPEN_GRIPPER = 1
    MOVE_TO_TARGET = 2
    CLOSE_GRIPPER = 3

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
        self.declare_parameter(
            'constrain_orientation', False)

        self.base_link = self.get_parameter('base_link').value
        self.end_effector = self.get_parameter('end_effector_link').value
        self.gripper_joint = self.get_parameter('gripper_joint').value
        self.gripper_open = self.get_parameter('gripper_open').value
        self.gripper_closed = self.get_parameter('gripper_closed').value
        self.target_z_offset = self.get_parameter('target_z_offset').value
        self.rejected_target_distance = self.get_parameter(
            'rejected_target_distance').value
        self.arm = MoveIt2(
            node=self,
            base_link=self.base_link,
            end_effector=self.end_effector,
            group_name=self.get_parameter('arm_group').value,
            constrain_orientation=self.get_parameter(
                'constrain_orientation').value,
        )
        self.gripper = ActionClient(
            self,
            FollowJointTrajectory,
            self.get_parameter('gripper_action').value,
        )
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.state = self.IDLE
        self.triggered = False
        self.validating_target = False
        self.gripper_done = True
        self.target_pose = None
        self.target_frame = None
        self.rejected_target = None

        self.create_subscription(
            PoseStamped, '/grasp_pose', self.grasp_callback, 10)
        self.create_timer(0.1, self.tick)
        self.get_logger().info('Nero seven-axis grasp executor ready')

    def grasp_callback(self, msg):
        if self.state != self.IDLE or self.validating_target:
            return
        position = msg.pose.position
        if self.rejected_target is not None:
            frame, x, y, z = self.rejected_target
            distance = ((position.x - x) ** 2 +
                        (position.y - y) ** 2 +
                        (position.z + self.target_z_offset - z) ** 2) ** 0.5
            if (msg.header.frame_id == frame and
                    distance < self.rejected_target_distance):
                return
        self.target_pose = Pose(
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
        self.target_frame = msg.header.frame_id
        self.validating_target = True
        self.arm.move_to_pose(
            self.target_pose, self.target_frame, plan_only=True)
        p = self.target_pose.position
        self.get_logger().info(
            f'Validating target in {self.target_frame}: '
            f'({p.x:.3f}, {p.y:.3f}, {p.z:.3f})')

    def send_gripper(self, position):
        self.gripper_done = False
        if not self.gripper.wait_for_server(timeout_sec=3.0):
            self.get_logger().error('Gripper trajectory action is unavailable')
            self.gripper_done = True
            return

        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = [self.gripper_joint]
        point = JointTrajectoryPoint()
        point.positions = [float(position)]
        point.time_from_start = Duration(sec=1)
        goal.trajectory.points = [point]
        future = self.gripper.send_goal_async(goal)
        future.add_done_callback(self.gripper_goal_response)

    def gripper_goal_response(self, future):
        goal_handle = future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error('Gripper goal rejected')
            self.gripper_done = True
            return
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.gripper_result_response)

    def gripper_result_response(self, future):
        del future
        self.gripper_done = True

    def tick(self):
        if self.state == self.IDLE and self.validating_target:
            if not self.arm.is_done():
                return
            self.validating_target = False
            if self.arm.success:
                self.rejected_target = None
                self.state = self.OPEN_GRIPPER
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
                self.state = self.MOVE_TO_TARGET
                self.triggered = False
            return
        if self.state == self.MOVE_TO_TARGET:
            if not self.triggered:
                self.arm.move_to_pose(self.target_pose, self.target_frame)
                self.triggered = True
            elif self.arm.is_done():
                if self.arm.success:
                    self.report_position_error()
                    self.state = self.CLOSE_GRIPPER
                else:
                    self.get_logger().error('Arm motion failed')
                    self.state = self.IDLE
                self.triggered = False
            return
        if self.state == self.CLOSE_GRIPPER:
            if not self.triggered:
                self.send_gripper(self.gripper_closed)
                self.triggered = True
            elif self.gripper_done:
                self.get_logger().info('Grasp sequence completed')
                self.state = self.IDLE
                self.triggered = False

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
