#!/usr/bin/env python3

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from tf2_geometry_msgs import do_transform_pose_stamped
from tf2_ros import Buffer, TransformException, TransformListener


class VirtualDepthCamera(Node):
    def __init__(self):
        super().__init__('virtual_depth_camera')
        self.declare_parameter('width', 640)
        self.declare_parameter('height', 480)
        self.declare_parameter('fx', 500.0)
        self.declare_parameter('fy', 500.0)
        self.declare_parameter('cx', 320.0)
        self.declare_parameter('cy', 240.0)
        self.declare_parameter('depth_value_mm', 300)
        self.declare_parameter('rate', 30.0)
        self.declare_parameter('camera_frame', 'camera_color_optical_frame')
        self.declare_parameter('base_frame', 'base_link')

        self.width = self.get_parameter('width').value
        self.height = self.get_parameter('height').value
        self.fx = self.get_parameter('fx').value
        self.fy = self.get_parameter('fy').value
        self.cx = self.get_parameter('cx').value
        self.cy = self.get_parameter('cy').value
        self.depth_fallback = self.get_parameter('depth_value_mm').value
        self.camera_frame = self.get_parameter('camera_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.marker_world = None

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_subscription(
            PoseStamped, '/virtual_marker_world', self.marker_callback, 10)
        self.depth_pub = self.create_publisher(
            Image, '/camera/camera/aligned_depth_to_color/image_raw', 10)
        self.info_pub = self.create_publisher(
            CameraInfo, '/camera/camera/color/camera_info', 10)

        self.info_msg = CameraInfo()
        self.info_msg.header.frame_id = self.camera_frame
        self.info_msg.width = self.width
        self.info_msg.height = self.height
        self.info_msg.k = [
            float(self.fx), 0.0, float(self.cx),
            0.0, float(self.fy), float(self.cy),
            0.0, 0.0, 1.0,
        ]
        self.info_msg.p = [
            float(self.fx), 0.0, float(self.cx), 0.0,
            0.0, float(self.fy), float(self.cy), 0.0,
            0.0, 0.0, 1.0, 0.0,
        ]

        rate = self.get_parameter('rate').value
        self.create_timer(1.0 / rate, self.publish_depth)
        self.create_timer(1.0, self.publish_info)

    def marker_callback(self, msg):
        self.marker_world = msg

    def marker_pixel(self):
        if self.marker_world is None:
            return None
        try:
            transform = self.tf_buffer.lookup_transform(
                self.camera_frame, self.base_frame, rclpy.time.Time())
        except TransformException:
            return None
        pose = do_transform_pose_stamped(self.marker_world, transform)
        x = pose.pose.position.x
        y = pose.pose.position.y
        z = pose.pose.position.z
        if z <= 0.01:
            return None
        u = int(self.cx + self.fx * x / z)
        v = int(self.cy + self.fy * y / z)
        if not (0 <= u < self.width and 0 <= v < self.height):
            return None
        return u, v, z

    def publish_info(self):
        self.info_msg.header.stamp = self.get_clock().now().to_msg()
        self.info_pub.publish(self.info_msg)

    def publish_depth(self):
        depth = np.full(
            (self.height, self.width), self.depth_fallback, dtype=np.uint16)
        marker = self.marker_pixel()
        if marker is not None:
            u, v, z = marker
            depth[max(0, v - 3):min(self.height, v + 4),
                  max(0, u - 3):min(self.width, u + 4)] = int(
                      np.clip(z * 1000.0, 1, 65535))

        msg = Image()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.camera_frame
        msg.height = self.height
        msg.width = self.width
        msg.encoding = '16UC1'
        msg.is_bigendian = False
        msg.step = self.width * 2
        msg.data = depth.tobytes()
        self.depth_pub.publish(msg)


def main():
    rclpy.init()
    node = VirtualDepthCamera()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
