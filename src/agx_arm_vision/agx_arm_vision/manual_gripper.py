#!/usr/bin/env python3

import rclpy
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import Float64, Bool
from trajectory_msgs.msg import JointTrajectoryPoint


class ManualGripper(Node):
    def __init__(self):
        super().__init__('manual_gripper')
        self.declare_parameter(
            'gripper_action', '/gripper_controller/follow_joint_trajectory')
        self.declare_parameter('gripper_joint', 'gripper')
        self.declare_parameter('max_width', 0.1)
        self.declare_parameter('min_width', 0.0)
        self.declare_parameter('effort', 1.0)
        self.declare_parameter('force_threshold', 0.5)
        self.declare_parameter('status_topic', '/feedback/gripper_status')

        self.gripper_joint = self.get_parameter('gripper_joint').value
        self.max_width = self.get_parameter('max_width').value
        self.min_width = self.get_parameter('min_width').value
        self.effort = self.get_parameter('effort').value
        self.force_threshold = self.get_parameter('force_threshold').value

        self.busy = False
        self.monitoring_grasp = False
        self.grasp_goal_handle = None
        self.grasp_force_threshold = 0.5
        self.current_force = 0.0
        self.current_width = 0.0

        self.gripper = ActionClient(
            self,
            FollowJointTrajectory,
            self.get_parameter('gripper_action').value,
        )

        self.create_subscription(
            Float64, '/manual_gripper', self.position_callback, 10)
        self.create_subscription(
            Float64, '/manual_gripper_grasp', self.grasp_callback, 10)
        self.grasp_result_pub = self.create_publisher(
            Bool, '/manual_gripper_result', 10)

        self._import_feedback_msg()
        if self._GripperStatusMsg is not None:
            self.create_subscription(
                self._GripperStatusMsg,
                self.get_parameter('status_topic').value,
                self.feedback_callback,
                10,
            )
            self.get_logger().info(
                'Gripper force feedback enabled')
        else:
            self.get_logger().warning(
                'Cannot import GripperStatus; force feedback disabled')

        self.get_logger().info(
            f'Manual gripper ready: range=[{self.min_width}, '
            f'{self.max_width}] m, force_threshold={self.force_threshold} N')

    def _import_feedback_msg(self):
        self._GripperStatusMsg = None
        try:
            from agx_arm_msgs.msg import GripperStatus
            self._GripperStatusMsg = GripperStatus
        except ImportError:
            pass

    def feedback_callback(self, msg):
        self.current_force = msg.force
        self.current_width = msg.width
        if self.monitoring_grasp and abs(msg.force) > self.grasp_force_threshold:
            self.monitoring_grasp = False
            self.get_logger().info(
                f'Object grasped! force={msg.force:.2f} N, '
                f'width={msg.width:.3f} m')
            self.grasp_result_pub.publish(Bool(data=True))
            self._cancel_grasp()

    def position_callback(self, msg):
        if self.busy:
            self.get_logger().warning('Gripper is busy; command ignored')
            return
        width = msg.data
        if not self.min_width <= width <= self.max_width:
            self.get_logger().warning(
                f'Width {width:.3f} outside [{self.min_width}, '
                f'{self.max_width}]; command rejected')
            return
        self._send_gripper(width)

    def grasp_callback(self, msg):
        if self.busy:
            self.get_logger().warning('Gripper is busy; command ignored')
            return
        threshold = msg.data
        if threshold <= 0 or threshold > 3.0:
            self.get_logger().warning(
                f'Force threshold {threshold:.1f} out of range (0, 3.0]')
            return
        self.grasp_force_threshold = threshold
        self.monitoring_grasp = True
        self.get_logger().info(
            f'Closing until force > {threshold:.1f} N ...')
        self._send_gripper(0.0)

    def _send_gripper(self, width):
        self.busy = True
        if not self.gripper.wait_for_server(timeout_sec=3.0):
            self.get_logger().error('Gripper action server unavailable')
            self.busy = False
            return
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = [self.gripper_joint]
        point = JointTrajectoryPoint()
        point.positions = [float(width)]
        point.time_from_start = Duration(sec=1)
        goal.trajectory.points = [point]
        self.get_logger().info(f'Moving gripper to {width:.3f} m')
        future = self.gripper.send_goal_async(goal)
        future.add_done_callback(self.goal_response)

    def goal_response(self, future):
        goal_handle = future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error('Gripper goal rejected')
            self.busy = False
            self.monitoring_grasp = False
            return
        self.grasp_goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_response)

    def result_response(self, future):
        del future
        self.grasp_goal_handle = None
        if self.monitoring_grasp:
            self.monitoring_grasp = False
            self.get_logger().info(
                f'Nothing grasped (gripper fully closed)')
            self.grasp_result_pub.publish(Bool(data=False))
        self.get_logger().info('Gripper done')
        self.busy = False

    def _cancel_grasp(self):
        if self.grasp_goal_handle is not None:
            self.grasp_goal_handle.cancel_goal_async()
            self.grasp_goal_handle = None
        self.busy = False


def main():
    rclpy.init()
    node = ManualGripper()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
