#!/usr/bin/env python3

import rclpy
from aruco_opencv_msgs.msg import ArucoDetection, MarkerPose
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from std_msgs.msg import ColorRGBA
from std_srvs.srv import Empty
from tf2_geometry_msgs import do_transform_pose_stamped
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker


class VirtualArucoPublisher(Node):
    def __init__(self):
        super().__init__('virtual_aruco_pub')
        self.declare_parameter('marker_id', 0)
        self.declare_parameter('x', 0.30)
        self.declare_parameter('y', 0.0)
        self.declare_parameter('z', 0.20)
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('camera_frame', 'camera_color_optical_frame')
        self.declare_parameter('rate', 10.0)

        self.marker_id = self.get_parameter('marker_id').value
        self.x = self.get_parameter('x').value
        self.y = self.get_parameter('y').value
        self.z = self.get_parameter('z').value
        self.base_frame = self.get_parameter('base_frame').value
        self.camera_frame = self.get_parameter('camera_frame').value

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.detection_pub = self.create_publisher(
            ArucoDetection, '/aruco_detections', 10)
        self.world_pub = self.create_publisher(
            PoseStamped, '/virtual_marker_world', 10)
        self.marker_pub = self.create_publisher(
            Marker, '/virtual_aruco_marker', 10)
        self.create_service(Empty, '/reset_virtual_marker', self.reset_marker)
        self.create_timer(
            1.0 / self.get_parameter('rate').value, self.publish_marker)

    def reset_marker(self, request, response):
        del request
        self.x = self.get_parameter('x').value
        self.y = self.get_parameter('y').value
        self.z = self.get_parameter('z').value
        return response

    def publish_marker(self):
        now = self.get_clock().now().to_msg()
        world_pose = PoseStamped()
        world_pose.header.stamp = now
        world_pose.header.frame_id = self.base_frame
        world_pose.pose.position.x = float(self.x)
        world_pose.pose.position.y = float(self.y)
        world_pose.pose.position.z = float(self.z)
        world_pose.pose.orientation.w = 1.0
        self.world_pub.publish(world_pose)

        square = Marker()
        square.header = world_pose.header
        square.ns = 'virtual_aruco'
        square.id = 0
        square.type = Marker.CUBE
        square.action = Marker.ADD
        square.pose = world_pose.pose
        square.scale.x = 0.05
        square.scale.y = 0.05
        square.scale.z = 0.002
        square.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=0.9)
        self.marker_pub.publish(square)

        try:
            transform = self.tf_buffer.lookup_transform(
                self.camera_frame, self.base_frame, rclpy.time.Time())
        except TransformException as exc:
            self.get_logger().warn(
                f'Virtual camera TF unavailable: {exc}',
                throttle_duration_sec=5.0,
            )
            return

        camera_pose = do_transform_pose_stamped(world_pose, transform)
        detection = ArucoDetection()
        detection.header.stamp = now
        detection.header.frame_id = self.camera_frame
        marker = MarkerPose()
        marker.marker_id = self.marker_id
        marker.pose = camera_pose.pose
        detection.markers = [marker]
        self.detection_pub.publish(detection)


def main():
    rclpy.init()
    node = VirtualArucoPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
