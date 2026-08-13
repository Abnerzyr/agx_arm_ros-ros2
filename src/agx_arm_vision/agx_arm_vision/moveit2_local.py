#!/usr/bin/env python3

from geometry_msgs.msg import Pose
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    AllowedCollisionEntry,
    AllowedCollisionMatrix,
    BoundingVolume,
    CollisionObject,
    Constraints,
    JointConstraint,
    MotionPlanRequest,
    OrientationConstraint,
    PositionConstraint,
)
from rclpy.action import ActionClient
from shape_msgs.msg import SolidPrimitive


class MoveIt2:
    def __init__(
            self, node, base_link, end_effector, group_name,
            action_name='/move_action', constrain_orientation=False,
            position_tolerance=0.01, orientation_tolerance=0.10):
        self.node = node
        self.base_link = base_link
        self.end_effector = end_effector
        self.group_name = group_name
        self.constrain_orientation = constrain_orientation
        self.position_tolerance = position_tolerance
        self.orientation_tolerance = orientation_tolerance
        self.action = ActionClient(node, MoveGroup, action_name)
        self.done = False
        self.success = False
        self.plan_only = False

    def _apply_scene_diff(self, goal, collision_objects, allowed_links):
        if not collision_objects:
            return
        scene = goal.planning_options.planning_scene_diff
        scene.is_diff = True
        scene.robot_state.is_diff = True
        acm = AllowedCollisionMatrix()
        acm.default_entry_names = list(allowed_links)
        for obj in collision_objects:
            co = CollisionObject()
            co.header.frame_id = self.base_link
            co.id = str(obj['id'])
            co.primitives = [SolidPrimitive(
                type=SolidPrimitive.BOX,
                dimensions=[
                    float(obj['size'][0]),
                    float(obj['size'][1]),
                    float(obj['size'][2]),
                ],
            )]
            co.primitive_poses = [Pose()]
            co.primitive_poses[0].position.x = float(obj['position'][0])
            co.primitive_poses[0].position.y = float(obj['position'][1])
            co.primitive_poses[0].position.z = float(obj['position'][2])
            co.operation = CollisionObject.ADD
            scene.world.collision_objects.append(co)
            acm.entry_names.append(str(obj['id']))
            acm.entry_values.append(AllowedCollisionEntry(
                enabled=[True] * len(allowed_links)))
        scene.allowed_collision_matrix = acm

    def move_to_pose(
            self, pose, frame_id=None, plan_only=False,
            velocity_scaling=0.2,
            collision_objects=None, allowed_links=None):
        frame_id = frame_id or self.base_link
        self.done = False
        self.success = False
        self.plan_only = plan_only

        goal = MoveGroup.Goal()
        goal.request = MotionPlanRequest()
        goal.request.group_name = self.group_name
        goal.request.start_state.is_diff = True
        goal.request.allowed_planning_time = 5.0 if plan_only else 3.0
        goal.request.max_velocity_scaling_factor = velocity_scaling
        goal.request.max_acceleration_scaling_factor = velocity_scaling
        goal.request.num_planning_attempts = 20
        goal.request.goal_constraints = [Constraints()]
        goal.planning_options.plan_only = plan_only
        goal.planning_options.planning_scene_diff.is_diff = True
        goal.planning_options.planning_scene_diff.robot_state.is_diff = True
        self._apply_scene_diff(goal, collision_objects, allowed_links)

        position = PositionConstraint()
        position.header.frame_id = frame_id
        position.link_name = self.end_effector
        position.constraint_region = BoundingVolume()
        position.constraint_region.primitives = [SolidPrimitive(
            type=SolidPrimitive.SPHERE,
            dimensions=[self.position_tolerance],
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
            orientation.absolute_x_axis_tolerance = self.orientation_tolerance
            orientation.absolute_y_axis_tolerance = self.orientation_tolerance
            orientation.absolute_z_axis_tolerance = self.orientation_tolerance
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

    def move_to_joints(
            self, positions, velocity_scaling=0.2,
            collision_objects=None, allowed_links=None):
        self.done = False
        self.success = False
        self.plan_only = False

        goal = MoveGroup.Goal()
        goal.request = MotionPlanRequest()
        goal.request.group_name = self.group_name
        goal.request.start_state.is_diff = True
        goal.request.allowed_planning_time = 3.0
        goal.request.max_velocity_scaling_factor = velocity_scaling
        goal.request.max_acceleration_scaling_factor = velocity_scaling
        goal.request.num_planning_attempts = 1
        goal.request.goal_constraints = [Constraints()]
        self._apply_scene_diff(goal, collision_objects, allowed_links)

        joint_names = ['joint1', 'joint2', 'joint3', 'joint4',
                       'joint5', 'joint6', 'joint7']
        for name, pos in zip(joint_names, positions):
            jc = JointConstraint()
            jc.joint_name = name
            jc.position = float(pos)
            jc.tolerance_above = 0.01
            jc.tolerance_below = 0.01
            jc.weight = 1.0
            goal.request.goal_constraints[0].joint_constraints.append(jc)

        if not self.action.wait_for_server(timeout_sec=5.0):
            self.node.get_logger().error('/move_action is unavailable')
            self.done = True
            return

        future = self.action.send_goal_async(goal)
        future.add_done_callback(self.goal_response)

    def is_done(self):
        return self.done
