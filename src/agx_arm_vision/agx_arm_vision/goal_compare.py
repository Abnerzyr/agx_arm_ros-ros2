#!/usr/bin/env python3
"""Compare MoveIt goals between executor and RViz Plan&Execute."""

import json
import math
import random
import rclpy
from geometry_msgs.msg import Pose, PoseStamped
from moveit_msgs.action import MoveGroup
from rclpy.action import ActionClient
from rclpy.node import Node
from tf2_ros import Buffer, TransformException, TransformListener


def _goal_to_dict(goal):
    g = goal.request
    info = {
        'group': g.group_name,
        'attempts': g.num_planning_attempts,
        'time': g.allowed_planning_time,
        'velocity': g.max_velocity_scaling_factor,
        'accel': g.max_acceleration_scaling_factor,
        'pos_tolerance': None,
        'ori_constraint': False,
        'start_diff': g.start_state.is_diff,
        'plan_only': goal.planning_options.plan_only,
        'scene_diff': goal.planning_options.planning_scene_diff.is_diff,
        'robot_diff': goal.planning_options.planning_scene_diff.robot_state.is_diff,
        'frame_id': None,
        'pos_weight': None,
    }
    if g.goal_constraints and g.goal_constraints[0].position_constraints:
        pc = g.goal_constraints[0].position_constraints[0]
        dims = pc.constraint_region.primitives[0].dimensions if pc.constraint_region.primitives else []
        info['pos_tolerance'] = dims[0] if dims else 'none'
        info['pos_weight'] = pc.weight
        info['frame_id'] = pc.header.frame_id
    if g.goal_constraints and g.goal_constraints[0].orientation_constraints:
        info['ori_constraint'] = True
        oc = g.goal_constraints[0].orientation_constraints[0]
        info['ori_tol_x'] = oc.absolute_x_axis_tolerance
        info['ori_tol_y'] = oc.absolute_y_axis_tolerance
        info['ori_tol_z'] = oc.absolute_z_axis_tolerance
    return info


class GoalCompareNode(Node):
    def __init__(self):
        super().__init__('goal_compare')
        self.declare_parameter('base_link', 'base_link')
        self.declare_parameter('end_effector', 'tcp_link')
        self.declare_parameter('jitter', 0.10)

        self.jitter = self.get_parameter('jitter').value
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.pose_pub = self.create_publisher(
            PoseStamped, '/test_target_pose', 10)
        self.captured_count = 0

        self._action_client = ActionClient(self, MoveGroup, '/move_action')
        self._orig_send_goal = self._action_client.send_goal_async
        self._action_client.send_goal_async = self._intercept_goal

        self.get_logger().info(
            'Goal compare ready. Intercepting /move_action goals')
        self.create_timer(2.0, self.send_oneshot)

    def send_oneshot(self):
        try:
            t = self.tf_buffer.lookup_transform(
                self.get_parameter('base_link').value,
                self.get_parameter('end_effector').value,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=1.0),
            )
        except TransformException:
            return
        pose = Pose()
        pose.position.x = t.transform.translation.x + random.uniform(
            -self.jitter, self.jitter)
        pose.position.y = t.transform.translation.y + random.uniform(
            -self.jitter, self.jitter)
        pose.position.z = t.transform.translation.z + random.uniform(
            -self.jitter, self.jitter)
        pose.orientation = t.transform.rotation

        p = pose.position
        self.get_logger().info(
            f'Target: ({p.x:.3f}, {p.y:.3f}, {p.z:.3f})')
        self.get_logger().info(
            '=== Now do Plan&Execute to this same point in RViz ===')

        pub_msg = PoseStamped()
        pub_msg.header.frame_id = self.get_parameter('base_link').value
        pub_msg.pose = pose
        self.pose_pub.publish(pub_msg)

        self.destroy_timer(self._timers[0])

    def capture_goal(self, goal):
        self.captured_count += 1
        info = _goal_to_dict(goal)
        self.get_logger().info(
            f'GOAL #{self.captured_count}: ' + json.dumps(info))
        with open(f'/tmp/goal_{self.captured_count}.json', 'w') as f:
            json.dump(info, f, indent=2)

    def _intercept_goal(self, goal):
        self.capture_goal(goal)
        return self._orig_send_goal(goal)


def main():
    rclpy.init()
    node = GoalCompareNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
