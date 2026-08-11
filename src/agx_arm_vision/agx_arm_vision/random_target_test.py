#!/usr/bin/env python3
"""Send random reachable poses to MoveIt for path planning testing."""

import math
import random

import numpy as np
import rclpy
from geometry_msgs.msg import Pose
from rclpy.node import Node
from tf2_ros import Buffer, TransformException, TransformListener

from agx_arm_vision.moveit2_local import MoveIt2


class RandomTargetNode(Node):
    def __init__(self):
        super().__init__('random_target_test')
        self.declare_parameter('base_link', 'base_link')
        self.declare_parameter('end_effector', 'tcp_link')
        self.declare_parameter('interval', 5.0)
        self.declare_parameter('jitter_range', 0.30)
        self.declare_parameter('min_z', 0.10)
        self.declare_parameter('max_z', 0.85)
        self.declare_parameter('max_radius', 0.70)
        self.declare_parameter('constrain_orientation', False)

        self.interval = self.get_parameter('interval').value
        self.jitter = self.get_parameter('jitter_range').value

        self.arm = MoveIt2(
            node=self,
            base_link=self.get_parameter('base_link').value,
            end_effector=self.get_parameter('end_effector').value,
            group_name='arm',
            constrain_orientation=self.get_parameter(
                'constrain_orientation').value,
        )
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.busy = False

        self.create_timer(self.interval, self.send_random_target)
        self.get_logger().info(
            f'Random target test ready, interval={self.interval}s, '
            f'jitter={self.jitter}m')

    def send_random_target(self):
        if self.busy:
            return
        try:
            t = self.tf_buffer.lookup_transform(
                self.get_parameter('base_link').value,
                self.get_parameter('end_effector').value,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=1.0),
            )
        except TransformException:
            return
        min_z = self.get_parameter('min_z').value
        max_z = self.get_parameter('max_z').value
        max_r = self.get_parameter('max_radius').value
        px = t.transform.translation.x + random.uniform(-self.jitter, self.jitter)
        py = t.transform.translation.y + random.uniform(-self.jitter, self.jitter)
        pz = t.transform.translation.z + random.uniform(-self.jitter, self.jitter)
        pz = max(min_z, min(max_z, pz))
        r = math.hypot(px, py)
        if r > max_r:
            scale = max_r / r
            px *= scale
            py *= scale
        pose = Pose()
        pose.position.x = px
        pose.position.y = py
        pose.position.z = pz
        pose.orientation = t.transform.rotation
        self.busy = True
        self.get_logger().info(
            f'Target: ({pose.position.x:.3f}, '
            f'{pose.position.y:.3f}, {pose.position.z:.3f})')
        self.arm.move_to_pose(pose)
        self._check_timer = self.create_timer(0.2, self.check_done)

    def check_done(self):
        if not self.arm.is_done():
            return
        if self.arm.success:
            self.get_logger().info('Reached')
        else:
            self.get_logger().warning('Failed')
        self.busy = False
        self._check_timer.cancel()


def main():
    rclpy.init()
    node = RandomTargetNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
