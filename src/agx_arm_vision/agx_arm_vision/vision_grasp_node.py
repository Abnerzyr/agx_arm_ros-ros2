#!/usr/bin/env python3

import numpy as np
import rclpy
from aruco_opencv_msgs.msg import ArucoDetection
from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped, PoseStamped
from image_geometry import PinholeCameraModel
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from tf2_geometry_msgs import do_transform_pose_stamped
from tf2_ros import Buffer, TransformException, TransformListener


class VisionGraspNode(Node):
    def __init__(self):
        super().__init__('vision_grasp_node')

        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter(
            'depth_topic', '/camera/camera/aligned_depth_to_color/image_raw')
        self.declare_parameter(
            'camera_info_topic', '/camera/camera/color/camera_info')
        self.declare_parameter('detection_topic', '/aruco_detections')
        self.declare_parameter('target_marker_id', 0)
        self.declare_parameter('use_depth', True)

        self.base_frame = self.get_parameter('base_frame').value
        self.target_marker_id = self.get_parameter('target_marker_id').value
        self.use_depth = self.get_parameter('use_depth').value

        self.bridge = CvBridge()
        self.model = PinholeCameraModel()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.depth_img = None
        self.depth_encoding = ''
        self.camera_info = None
        self.log_count = 0

        self.create_subscription(
            ArucoDetection,
            self.get_parameter('detection_topic').value,
            self.aruco_callback,
            10,
        )
        self.create_subscription(
            Image,
            self.get_parameter('depth_topic').value,
            self.depth_callback,
            10,
        )
        self.create_subscription(
            CameraInfo,
            self.get_parameter('camera_info_topic').value,
            self.info_callback,
            10,
        )

        self.grasp_pub = self.create_publisher(
            PoseStamped, '/grasp_pose', 10)
        self.center_pub = self.create_publisher(
            PointStamped, '/aruco_center', 10)
        self.get_logger().info(
            f'Vision ready: marker={self.target_marker_id}, '
            f'base={self.base_frame}')

    def info_callback(self, msg):
        self.model.fromCameraInfo(msg)
        self.camera_info = msg

    def depth_callback(self, msg):
        self.depth_img = self.bridge.imgmsg_to_cv2(msg, 'passthrough')
        self.depth_encoding = msg.encoding

    def _depth_meters(self, value):
        if not np.isfinite(value) or value <= 0:
            return None
        if self.depth_encoding in ('32FC1', '64FC1'):
            return float(value)
        return float(value) / 1000.0

    def aruco_callback(self, msg):
        marker = next(
            (item for item in msg.markers
             if item.marker_id == self.target_marker_id),
            None,
        )
        if marker is None:
            return

        x = float(marker.pose.position.x)
        y = float(marker.pose.position.y)
        z = float(marker.pose.position.z)

        has_depth = self.depth_img is not None and self.camera_info is not None
        if self.use_depth and has_depth:
            u = int(
                self.model.cx() + x * self.model.fx() / z) if z != 0 else -1
            v = int(
                self.model.cy() + y * self.model.fy() / z) if z != 0 else -1
            pixel_is_valid = (
                0 <= u < self.depth_img.shape[1]
                and 0 <= v < self.depth_img.shape[0]
            )
            if pixel_is_valid:
                depth = self._depth_meters(self.depth_img[v, u])
                if depth is not None and z != 0:
                    scale = depth / z
                    x *= scale
                    y *= scale
                    z = depth

        center = PointStamped()
        center.header = msg.header
        center.point.x = x
        center.point.y = y
        center.point.z = z
        self.center_pub.publish(center)

        try:
            transform = self.tf_buffer.lookup_transform(
                self.base_frame,
                msg.header.frame_id,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=1.0),
            )
        except TransformException as exc:
            self.get_logger().warn(
                f'Camera-to-base TF unavailable: {exc}',
                throttle_duration_sec=5.0,
            )
            return

        camera_pose = PoseStamped()
        camera_pose.header = msg.header
        camera_pose.pose.position.x = x
        camera_pose.pose.position.y = y
        camera_pose.pose.position.z = z
        camera_pose.pose.orientation = marker.pose.orientation
        base_pose = do_transform_pose_stamped(camera_pose, transform)
        self.grasp_pub.publish(base_pose)

        self.log_count += 1
        if self.log_count % 30 == 1:
            p = base_pose.pose.position
            self.get_logger().info(
                f'ArUco {marker.marker_id} in {self.base_frame}: '
                f'({p.x:.3f}, {p.y:.3f}, {p.z:.3f})')


def main():
    rclpy.init()
    node = VisionGraspNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
