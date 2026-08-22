#!/usr/bin/env python3

from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import Pose
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    BoundingVolume,
    CollisionObject,
    Constraints,
    JointConstraint,
    MotionPlanRequest,
    OrientationConstraint,
    PositionConstraint,
)
from moveit_msgs.srv import ApplyPlanningScene, GetCartesianPath
from rclpy.action import ActionClient
from shape_msgs.msg import SolidPrimitive


class MoveIt2:
    def __init__(
            self, node, base_link, end_effector, group_name,
            action_name='move_action', constrain_orientation=False,
            position_tolerance=0.01, orientation_tolerance=0.10):
        self.node = node
        self.base_link = base_link
        self.end_effector = end_effector
        self.group_name = group_name
        self.constrain_orientation = constrain_orientation
        self.position_tolerance = position_tolerance
        self.orientation_tolerance = orientation_tolerance
        self.action = ActionClient(node, MoveGroup, action_name)
        self._apply_scene_client = node.create_client(
            ApplyPlanningScene, 'apply_planning_scene')
        self._cartesian_client = node.create_client(
            GetCartesianPath, 'compute_cartesian_path')
        self._traj_client = ActionClient(
            node, FollowJointTrajectory,
            'arm_controller/follow_joint_trajectory')
        self.done = False
        self.success = False
        self.plan_only = False

    # ------------------------------------------------------------------
    # Collision object add / remove via /apply_planning_scene.
    # The target box is added to the planning scene as a plain world
    # collision object (no AllowedCollisionMatrix manipulation).
    # ------------------------------------------------------------------
    def apply_collision_object(
            self, collision_object, add=True, callback=None):
        if not self._apply_scene_client.service_is_ready():
            self.node.get_logger().error(
                '/apply_planning_scene service not available')
            return False
        req = ApplyPlanningScene.Request()
        req.scene.is_diff = True
        co = CollisionObject()
        co.header.frame_id = self.base_link
        co.id = str(collision_object['id'])
        if add:
            co.primitives = [SolidPrimitive(
                type=SolidPrimitive.BOX,
                dimensions=[
                    float(collision_object['size'][0]),
                    float(collision_object['size'][1]),
                    float(collision_object['size'][2]),
                ],
            )]
            co.primitive_poses = [Pose()]
            co.primitive_poses[0].position.x = float(
                collision_object['position'][0])
            co.primitive_poses[0].position.y = float(
                collision_object['position'][1])
            co.primitive_poses[0].position.z = float(
                collision_object['position'][2])
            quat = collision_object.get(
                'orientation', (0.0, 0.0, 0.0, 1.0))
            co.primitive_poses[0].orientation.x = float(quat[0])
            co.primitive_poses[0].orientation.y = float(quat[1])
            co.primitive_poses[0].orientation.z = float(quat[2])
            co.primitive_poses[0].orientation.w = float(quat[3])
        co.operation = CollisionObject.ADD if add else CollisionObject.REMOVE
        req.scene.world.collision_objects.append(co)
        future = self._apply_scene_client.call_async(req)
        if callback is not None:
            future.add_done_callback(callback)
        return True

    # ------------------------------------------------------------------
    # Cartesian straight-line path (compute_cartesian_path + execute).
    # ------------------------------------------------------------------
    def move_cartesian_to(
            self, waypoints, frame_id=None, max_step=0.01,
            jump_threshold=0.0, avoid_collisions=True):
        """Plan a straight-line Cartesian path through `waypoints` and execute
        it asynchronously (sets done/success like move_to_pose)."""
        frame_id = frame_id or self.base_link
        self.done = False
        self.success = False
        self.plan_only = False
        if not self._cartesian_client.service_is_ready():
            self.node.get_logger().error(
                '/compute_cartesian_path service not available')
            self.done = True
            return
        req = GetCartesianPath.Request()
        req.header.frame_id = frame_id
        req.group_name = self.group_name
        req.link_name = self.end_effector
        req.start_state.is_diff = True
        req.max_step = max_step
        req.jump_threshold = jump_threshold
        req.avoid_collisions = avoid_collisions
        for wp in waypoints:
            req.waypoints.append(wp)
        future = self._cartesian_client.call_async(req)

        def on_path(f):
            try:
                res = f.result()
            except Exception:
                res = None
            if (res is None or res.solution is None
                    or res.fraction < 1.0 - 1e-6):
                frac = getattr(res, 'fraction', None)
                self.node.get_logger().error(
                    f'Cartesian path failed (fraction={frac})')
                self.done = True
                self.success = False
                return
            self._execute_trajectory(res.solution)

        future.add_done_callback(on_path)

    def _execute_trajectory(self, robot_trajectory):
        if not self._traj_client.wait_for_server(timeout_sec=5.0):
            self.node.get_logger().error(
                '/arm_controller/follow_joint_trajectory unavailable')
            self.done = True
            self.success = False
            return
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = robot_trajectory.joint_trajectory
        future = self._traj_client.send_goal_async(goal)
        future.add_done_callback(self._traj_goal_response)

    def _traj_goal_response(self, future):
        goal_handle = future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.node.get_logger().error(
                'FollowJointTrajectory goal rejected')
            self.done = True
            self.success = False
            return
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._traj_result_response)

    def _traj_result_response(self, future):
        result = future.result().result
        self.success = (result.error_code == 0)
        if not self.success:
            self.node.get_logger().error(
                'FollowJointTrajectory failed with code '
                f'{result.error_code}')
        self.done = True

    # ------------------------------------------------------------------
    # MoveGroup actions
    # ------------------------------------------------------------------
    def move_to_pose(
            self, pose, frame_id=None, plan_only=False,
            velocity_scaling=0.2, constrain_orientation=None,
            orientation_tolerance=None):
        frame_id = frame_id or self.base_link
        if constrain_orientation is None:
            constrain_orientation = self.constrain_orientation
        if orientation_tolerance is None:
            orientation_tolerance = self.orientation_tolerance
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

        if constrain_orientation:
            orientation = OrientationConstraint()
            orientation.header.frame_id = frame_id
            orientation.link_name = self.end_effector
            orientation.orientation = pose.orientation
            orientation.absolute_x_axis_tolerance = orientation_tolerance
            orientation.absolute_y_axis_tolerance = orientation_tolerance
            orientation.absolute_z_axis_tolerance = orientation_tolerance
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
            self, positions, velocity_scaling=0.2):
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
