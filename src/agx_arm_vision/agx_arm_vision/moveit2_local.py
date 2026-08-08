#!/usr/bin/env python3

from geometry_msgs.msg import Pose
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    BoundingVolume,
    Constraints,
    MotionPlanRequest,
    OrientationConstraint,
    PositionConstraint,
)
from rclpy.action import ActionClient
from shape_msgs.msg import SolidPrimitive


class MoveIt2:
    def __init__(
            self, node, base_link, end_effector, group_name,
            action_name='/move_action', constrain_orientation=False):
        self.node = node
        self.base_link = base_link
        self.end_effector = end_effector
        self.group_name = group_name
        self.constrain_orientation = constrain_orientation
        self.action = ActionClient(node, MoveGroup, action_name)
        self.done = False
        self.success = False
        self.plan_only = False

    def move_to_pose(self, pose, frame_id=None, plan_only=False):
        frame_id = frame_id or self.base_link
        self.done = False
        self.success = False
        self.plan_only = plan_only

        goal = MoveGroup.Goal()
        goal.request = MotionPlanRequest()
        goal.request.group_name = self.group_name
        goal.request.start_state.is_diff = True
        goal.request.allowed_planning_time = 3.0 if plan_only else 10.0
        goal.request.max_velocity_scaling_factor = 0.2
        goal.request.max_acceleration_scaling_factor = 0.2
        goal.request.num_planning_attempts = 20
        goal.request.goal_constraints = [Constraints()]
        goal.planning_options.plan_only = plan_only
        goal.planning_options.planning_scene_diff.is_diff = True
        goal.planning_options.planning_scene_diff.robot_state.is_diff = True

        position = PositionConstraint()
        position.header.frame_id = frame_id
        position.link_name = self.end_effector
        position.constraint_region = BoundingVolume()
        position.constraint_region.primitives = [SolidPrimitive(
            type=SolidPrimitive.SPHERE,
            dimensions=[0.01],
        )]
        position.constraint_region.primitive_poses = [Pose()]
        position.constraint_region.primitive_poses[0].position = pose.position
        position.weight = 1.0
        goal.request.goal_constraints[0].position_constraints = [
            position]

        if self.constrain_orientation:
            orientation = OrientationConstraint()
            orientation.header.frame_id = frame_id
            orientation.link_name = self.end_effector
            orientation.orientation = pose.orientation
            orientation.absolute_x_axis_tolerance = 0.15
            orientation.absolute_y_axis_tolerance = 0.15
            orientation.absolute_z_axis_tolerance = 0.15
            orientation.weight = 1.0
            goal.request.goal_constraints[0].orientation_constraints = [
                orientation]

        if not self.action.wait_for_server(timeout_sec=5.0):
            self.node.get_logger().error('/move_action is unavailable')
            self.done = True
            return

        future = self.action.send_goal_async(goal)
        future.add_done_callback(self.goal_response)

    def goal_response(self, future):
        goal_handle = future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.node.get_logger().error('MoveIt goal rejected')
            self.done = True
            return
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_response)

    def result_response(self, future):
        result = future.result().result
        self.success = result.error_code.val == 1
        if not self.success:
            message = f'MoveIt failed with code {result.error_code.val}'
            if self.plan_only:
                self.node.get_logger().warning(message)
            else:
                self.node.get_logger().error(message)
        self.done = True

    def is_done(self):
        return self.done
