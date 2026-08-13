#!/usr/bin/env python3

import rclpy
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

from agx_arm_msgs.msg import GripperStatus


class MockGripper(Node):
    def __init__(self):
        super().__init__('mock_gripper')
        self.declare_parameter('simulate_object', True)
        self.declare_parameter('object_width', 0.03)
        self.declare_parameter('grasp_force', 2.0)
        self.declare_parameter(
            'gripper_action', '/gripper_controller/follow_joint_trajectory')
        self.declare_parameter('gripper_joint', 'gripper')
        self.declare_parameter('trajectory_duration', 1.0)
        self.declare_parameter('press_tolerance', 0.002)

        self.simulate_object = self.get_parameter(
            'simulate_object').value
        self.object_width = self.get_parameter('object_width').value
        self.grasp_force = self.get_parameter('grasp_force').value
        self.joint_name = self.get_parameter('gripper_joint').value
        self.traj_duration = self.get_parameter(
            'trajectory_duration').value
        self.press_tol = self.get_parameter('press_tolerance').value

        self.width = None
        self.force = 0.0
        self.target = None
        self.goal_handle = None
        self._pressed = False
        self._pending = None

        self.gripper_client = ActionClient(
            self,
            FollowJointTrajectory,
            self.get_parameter('gripper_action').value,
        )

        self.create_subscription(
            JointState, '/control/gripper_target',
            self.target_callback, 10)
        self.create_subscription(
            JointState, '/control/joint_states',
            self.joint_states_callback, 10)
        self.status_pub = self.create_publisher(
            GripperStatus, '/feedback/gripper_status', 10)
        self.create_timer(0.02, self.step)
        self.get_logger().info(
            f'Mock gripper ready (simulate_object={self.simulate_object}, '
            f'object_width={self.object_width:.3f} m)')

    def target_callback(self, msg):
        if not msg.position:
            return
        self.target = float(msg.position[0])
        self._pressed = False
        if self.width is None:
            self._pending = self.target
            return
        self._send_goal(self.target)

    def joint_states_callback(self, msg):
        if self.joint_name not in msg.name:
            return
        idx = msg.name.index(self.joint_name)
        if idx < len(msg.position):
            self.width = float(msg.position[idx])
            if self._pending is not None:
                pending = self._pending
                self._pending = None
                self._send_goal(pending)

    def step(self):
        if (self.simulate_object and not self._pressed
                and self.target is not None
                and self.width is not None
                and self.width - self.target > self.press_tol
                and self.width <= self.object_width + self.press_tol):
            self._pressed = True
            self._cancel_goal()
            self.get_logger().info(
                f'Object contact at width={self.width:.3f} m, '
                f'force={self.grasp_force:.1f} N')
        self.force = self.grasp_force if self._pressed else 0.0
        self._publish()

    def _send_goal(self, width):
        if not self.gripper_client.wait_for_server(timeout_sec=3.0):
            self.get_logger().warn('Gripper action server unavailable')
            return
        start = self.width if self.width is not None else float(width)
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = [self.joint_name]
        start_point = JointTrajectoryPoint()
        start_point.positions = [float(start)]
        start_point.time_from_start = Duration(sec=0)
        end_point = JointTrajectoryPoint()
        end_point.positions = [float(width)]
        end_point.time_from_start = Duration(sec=int(self.traj_duration))
        goal.trajectory.points = [start_point, end_point]
        future = self.gripper_client.send_goal_async(goal)
        future.add_done_callback(self._goal_response)

    def _goal_response(self, future):
        handle = future.result()
        if handle is None or not handle.accepted:
            self.get_logger().warn('Gripper goal rejected')
            return
        self.goal_handle = handle

    def _cancel_goal(self):
        if self.goal_handle is not None:
            self.goal_handle.cancel_goal_async()
            self.goal_handle = None

    def _publish(self):
        if self.width is None:
            return
        msg = GripperStatus()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.width = float(self.width)
        msg.force = float(self.force)
        self.status_pub.publish(msg)


def main():
    rclpy.init()
    node = MockGripper()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
