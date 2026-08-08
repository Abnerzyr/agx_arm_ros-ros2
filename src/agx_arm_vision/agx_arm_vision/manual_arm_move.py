#!/usr/bin/env python3

import math

import rclpy
from geometry_msgs.msg import Pose, Vector3
from rclpy.node import Node
from tf2_ros import Buffer, TransformException, TransformListener

from agx_arm_vision.moveit2_local import MoveIt2


class ManualArmMove(Node):
    READY = 0
    VALIDATING = 1
    EXECUTING = 2

    def __init__(self):
        super().__init__('manual_arm_move')
        self.declare_parameter('base_link', 'base_link')
        self.declare_parameter('end_effector_link', 'tcp_link')
        self.declare_parameter('arm_group', 'arm')
        self.declare_parameter('max_step', 0.03)
        self.declare_parameter('min_z', 0.10)
        self.declare_parameter('max_z', 0.90)
        self.declare_parameter('max_radius', 0.75)
        self.declare_parameter('velocity_scaling', 0.05)

        self.base_link = self.get_parameter('base_link').value
        self.end_effector = self.get_parameter('end_effector_link').value
        self.max_step = self.get_parameter('max_step').value
        self.min_z = self.get_parameter('min_z').value
        self.max_z = self.get_parameter('max_z').value
        self.max_radius = self.get_parameter('max_radius').value
        self.velocity_scaling = self.get_parameter('velocity_scaling').value

        self.arm = MoveIt2(
            node=self,
            base_link=self.base_link,
            end_effector=self.end_effector,
            group_name=self.get_parameter('arm_group').value,
            constrain_orientation=True,
            position_tolerance=0.002,
            orientation_tolerance=0.05,
        )
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.state = self.READY
        self.target_pose = None

        self.create_subscription(
            Vector3, '/manual_arm_delta', self.delta_callback, 10)
        self.create_timer(0.1, self.tick)
        self.get_logger().info(
            f'Manual arm move ready: max_step={self.max_step:.3f} m, '
            f'velocity_scaling={self.velocity_scaling:.2f}')

    def delta_callback(self, msg):
        if self.state != self.READY:
            self.get_logger().warning('Arm is busy; command ignored')
            return

        step = math.sqrt(msg.x ** 2 + msg.y ** 2 + msg.z ** 2)
        if step < 1e-6:
            self.get_logger().warning('Zero displacement; command ignored')
            return
        if step > self.max_step:
            self.get_logger().warning(
                f'Displacement {step:.3f} m exceeds '
                f'max_step {self.max_step:.3f} m; command rejected')
            return

        try:
            transform = self.tf_buffer.lookup_transform(
                self.base_link,
                self.end_effector,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=1.0),
            )
        except TransformException as exc:
            self.get_logger().error(f'Current TCP pose unavailable: {exc}')
            return

        current = transform.transform
        target = Pose()
        target.position.x = current.translation.x + msg.x
        target.position.y = current.translation.y + msg.y
        target.position.z = current.translation.z + msg.z
        target.orientation = current.rotation

        radius = math.hypot(target.position.x, target.position.y)
        if not self.min_z <= target.position.z <= self.max_z:
            self.get_logger().warning(
                f'Target z={target.position.z:.3f} m is outside '
                f'[{self.min_z:.3f}, {self.max_z:.3f}] m; command rejected')
            return
        if radius > self.max_radius:
            self.get_logger().warning(
                f'Target radius={radius:.3f} m exceeds '
                f'{self.max_radius:.3f} m; command rejected')
            return

        self.target_pose = target
        self.state = self.VALIDATING
        self.arm.move_to_pose(
            target,
            self.base_link,
            plan_only=True,
            velocity_scaling=self.velocity_scaling,
        )
        self.get_logger().info(
            f'Validating delta ({msg.x:.3f}, {msg.y:.3f}, {msg.z:.3f}) m')

    def tick(self):
        if self.state == self.VALIDATING and self.arm.is_done():
            if not self.arm.success:
                self.get_logger().warning(
                    'Manual target is unreachable; no motion executed')
                self.state = self.READY
                self.target_pose = None
                return
            self.get_logger().info('Target is reachable; executing motion')
            self.state = self.EXECUTING
            self.arm.move_to_pose(
                self.target_pose,
                self.base_link,
                velocity_scaling=self.velocity_scaling,
            )
            return

        if self.state == self.EXECUTING and self.arm.is_done():
            if self.arm.success:
                self.get_logger().info('Manual motion completed')
            else:
                self.get_logger().error('Manual motion failed')
            self.state = self.READY
            self.target_pose = None


def main():
    rclpy.init()
    node = ManualArmMove()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
