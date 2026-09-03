#!/usr/bin/env python3

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker


class GraspTargetMarker(Node):
    def __init__(self):
        super().__init__('grasp_target_marker')
        self.create_subscription(
            PoseStamped, 'grasp_pose_display', self.pose_callback, 10)
        self.marker_pub = self.create_publisher(
            Marker, 'grasp_target_marker', 10)
        self.get_logger().info('Grasp target marker ready')

    def pose_callback(self, msg):
        sphere = Marker()
        sphere.header.frame_id = msg.header.frame_id
        sphere.header.stamp = self.get_clock().now().to_msg()
        sphere.ns = 'grasp_target'
        sphere.id = 0
        sphere.type = Marker.SPHERE
        sphere.action = Marker.ADD
        sphere.pose.position = msg.pose.position
        sphere.pose.orientation.w = 1.0
        sphere.scale.x = 0.03
        sphere.scale.y = 0.03
        sphere.scale.z = 0.03
        sphere.color = ColorRGBA(r=1.0, g=0.2, b=0.2, a=0.9)
        self.marker_pub.publish(sphere)

        arrow = Marker()
        arrow.header.frame_id = msg.header.frame_id
        arrow.header.stamp = sphere.header.stamp
        arrow.ns = 'grasp_target'
        arrow.id = 1
        arrow.type = Marker.ARROW
        arrow.action = Marker.ADD
        arrow.pose = msg.pose
        arrow.scale.x = 0.08
        arrow.scale.y = 0.01
        arrow.scale.z = 0.02
        arrow.color = ColorRGBA(r=1.0, g=0.6, b=0.0, a=0.9)
        self.marker_pub.publish(arrow)


def main():
    rclpy.init()
    node = GraspTargetMarker()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
