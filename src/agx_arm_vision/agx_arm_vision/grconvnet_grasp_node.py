#!/usr/bin/env python3

import math
import os

import cv2
import numpy as np
import rclpy
import torch
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped
from image_geometry import PinholeCameraModel
from rclpy.node import Node
from scipy.spatial.transform import Rotation as R
from sensor_msgs.msg import CameraInfo, Image, PointCloud2, PointField
from skimage.filters import gaussian
from std_msgs.msg import Header
from tf2_geometry_msgs import do_transform_pose_stamped
from tf2_ros import Buffer, TransformException, TransformListener


class GRConvNetGraspNode(Node):
    def __init__(self):
        super().__init__('grconvnet_grasp_node')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter(
            'depth_topic', '/camera/camera/aligned_depth_to_color/image_raw')
        self.declare_parameter(
            'rgb_topic', '/camera/camera/color/image_raw')
        self.declare_parameter(
            'info_topic', '/camera/camera/color/camera_info')
        self.declare_parameter('quality_threshold', 0.5)
        self.declare_parameter('input_size', 224)

        self.base_frame = self.get_parameter('base_frame').value
        self.quality_threshold = self.get_parameter('quality_threshold').value
        self.input_size = self.get_parameter('input_size').value

        self.bridge = CvBridge()
        self.model_cam = PinholeCameraModel()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.depth_img = None
        self.rgb_img = None
        self.camera_info = None

        self.model = self._load_model()
        self.get_logger().info('GR-ConvNet model loaded')

        self.create_subscription(
            Image, self.get_parameter('depth_topic').value,
            self.depth_callback, 10)
        self.create_subscription(
            Image, self.get_parameter('rgb_topic').value,
            self.rgb_callback, 10)
        self.create_subscription(
            CameraInfo, self.get_parameter('info_topic').value,
            self.info_callback, 10)
        self.grasp_pub = self.create_publisher(
            PoseStamped, '/grasp_pose', 10)
        self.cloud_pub = self.create_publisher(
            PointCloud2, '/grconvnet/points', 10)
        self.quality_pub = self.create_publisher(
            Image, '/grconvnet/quality_map', 10)
        self.create_timer(0.5, self.process)
        self.get_logger().info('GR-ConvNet grasp node ready')

    def _load_model(self):
        pkg_dir = os.path.dirname(os.path.abspath(__file__))
        from agx_arm_vision.models.grconvnet3 import GenerativeResnet
        model = GenerativeResnet(
            input_channels=4, output_channels=1,
            channel_size=32, dropout=False)
        weight_file = os.path.join(pkg_dir, 'models', 'grconvnet_statedict.pt')
        state = torch.load(weight_file, map_location='cpu', weights_only=True)
        model.load_state_dict(state)
        model.eval()
        return model

    def info_callback(self, msg):
        self.model_cam.fromCameraInfo(msg)
        self.camera_info = msg

    def depth_callback(self, msg):
        self.depth_img = self.bridge.imgmsg_to_cv2(msg, 'passthrough')

    def rgb_callback(self, msg):
        self.rgb_img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

    def _depth_meters(self, img):
        if img.dtype == np.uint16:
            return img.astype(np.float32) * 0.001
        return img.astype(np.float32)

    def process(self):
        if (self.depth_img is None or self.rgb_img is None
                or self.camera_info is None):
            return
        depth = self._depth_meters(self.depth_img)
        rgb = self.rgb_img
        h, w = depth.shape
        crop_size = min(h, w)
        start_h = (h - crop_size) // 2
        start_w = (w - crop_size) // 2
        depth_crop = depth[start_h:start_h + crop_size,
                           start_w:start_w + crop_size].copy()
        rgb_crop = rgb[start_h:start_h + crop_size,
                       start_w:start_w + crop_size].copy()
        mask = (depth_crop == 0) | (~np.isfinite(depth_crop))
        depth_crop[mask] = 0.0
        if mask.any():
            depth_crop = cv2.inpaint(
                depth_crop, mask.astype(np.uint8), 3, cv2.INPAINT_TELEA)
        depth_resized = cv2.resize(depth_crop, (self.input_size, self.input_size))
        rgb_resized = cv2.resize(rgb_crop, (self.input_size, self.input_size))
        depth_norm = depth_resized - depth_resized.mean()
        depth_norm = depth_norm / max(depth_norm.std(), 1e-6)
        depth_norm = np.clip(depth_norm, -1.0, 1.0)
        rgb_norm = rgb_resized.astype(np.float32) / 255.0
        rgb_norm = (rgb_norm - np.array([0.485, 0.456, 0.406])) / np.array([0.229, 0.224, 0.225])
        rgb_norm = np.clip(rgb_norm, -3.0, 3.0)
        x = np.concatenate([rgb_norm.transpose(2, 0, 1), depth_norm[np.newaxis, :, :]], axis=0)
        tensor = torch.from_numpy(x).float().unsqueeze(0)
        with torch.no_grad():
            quality, cos_ang, sin_ang, width = self.model(tensor)
        quality_np = gaussian(quality.squeeze().cpu().numpy(), 2.0, preserve_range=True)
        self._publish_quality(quality_np)
        ang_np = (torch.atan2(sin_ang, cos_ang) / 2.0).squeeze().cpu().numpy()
        ang_np = gaussian(ang_np, 2.0, preserve_range=True)
        width_np = width.squeeze().cpu().numpy() * 150.0
        width_np = gaussian(width_np, 1.0, preserve_range=True)
        best_pixel = np.unravel_index(np.argmax(quality_np), quality_np.shape)
        best_score = quality_np[best_pixel]
        if best_score < self.quality_threshold:
            return
        v, u = best_pixel
        angle = ang_np[v, u]
        grasp_width = max(0.0, min(0.1, width_np[v, u] * 0.001))
        scale = crop_size / self.input_size
        orig_u = int(u * scale + start_w)
        orig_v = int(v * scale + start_h)
        if orig_u < 0 or orig_u >= w or orig_v < 0 or orig_v >= h:
            return
        z = depth[orig_v, orig_u]
        if z <= 0.05 or z > 2.0:
            return
        fx = self.model_cam.fx()
        fy = self.model_cam.fy()
        cx = self.model_cam.cx()
        cy = self.model_cam.cy()
        x_pos = (orig_u - cx) * z / fx
        y_pos = (cy - orig_v) * z / fy
        self._publish_cloud(depth)
        self._publish(x_pos, y_pos, z, angle, grasp_width, best_score)

    def _publish(self, x, y, z, angle, width, score):
        try:
            transform = self.tf_buffer.lookup_transform(
                self.base_frame,
                self.camera_info.header.frame_id,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=1.0),
            )
        except TransformException as exc:
            self.get_logger().warn(f'TF: {exc}', throttle_duration_sec=5.0)
            return
        grasp_rot = R.from_euler('z', angle)
        pose = PoseStamped()
        pose.header.frame_id = self.camera_info.header.frame_id
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.position.z = float(z)
        q = grasp_rot.as_quat()
        pose.pose.orientation.x = q[0]
        pose.pose.orientation.y = q[1]
        pose.pose.orientation.z = q[2]
        pose.pose.orientation.w = q[3]
        base_pose = do_transform_pose_stamped(pose, transform)
        base_pose.pose.position.y = -base_pose.pose.position.y
        self.grasp_pub.publish(base_pose)
        self.get_logger().info(
            f'Grasp: ({base_pose.pose.position.x:.3f}, '
            f'{base_pose.pose.position.y:.3f}, '
            f'{base_pose.pose.position.z:.3f}) '
            f'angle={math.degrees(angle):.1f}° '
            f'width={width:.3f}m score={score:.2f}')

    def _publish_cloud(self, depth):
        try:
            t = self.tf_buffer.lookup_transform(
                self.base_frame,
                self.camera_info.header.frame_id,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=1.0),
            )
        except TransformException:
            return
        h, w = depth.shape
        fx = self.model_cam.fx()
        fy = self.model_cam.fy()
        cx = self.model_cam.cx()
        cy = self.model_cam.cy()
        ds = 4
        u = np.arange(0, w, ds)
        v = np.arange(0, h, ds)
        uu, vv = np.meshgrid(u, v)
        z = depth[vv, uu]
        valid = (z > 0.05) & (z < 2.0) & np.isfinite(z)
        z = z[valid]
        uu = uu[valid]
        vv = vv[valid]
        x = (uu - cx) * z / fx
        y = (cy - vv) * z / fy
        points = np.stack([x, y, z], axis=1)
        q = t.transform.rotation
        mat = R.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
        tx, ty, tz = t.transform.translation.x, t.transform.translation.y, t.transform.translation.z
        p = (mat @ points.T).T + np.array([tx, ty, tz])
        p[:, 1] = -p[:, 1]
        if len(p) > 10000:
            idx = np.random.choice(len(p), 10000, replace=False)
            p = p[idx]
        hdr = Header()
        hdr.frame_id = self.base_frame
        hdr.stamp = self.get_clock().now().to_msg()
        cloud = PointCloud2()
        cloud.header = hdr
        cloud.height = 1
        cloud.width = len(p)
        cloud.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        cloud.point_step = 12
        cloud.row_step = cloud.point_step * cloud.width
        cloud.is_bigendian = False
        cloud.is_dense = True
        cloud.data = p.astype(np.float32).tobytes()
        self.cloud_pub.publish(cloud)

    def _publish_quality(self, quality_np):
        q_min, q_max = quality_np.min(), quality_np.max()
        if q_max - q_min < 1e-6:
            return
        q_norm = ((quality_np - q_min) / (q_max - q_min) * 255).astype(np.uint8)
        q_color = cv2.applyColorMap(q_norm, cv2.COLORMAP_JET)
        msg = self.bridge.cv2_to_imgmsg(q_color, encoding='bgr8')
        msg.header.stamp = self.get_clock().now().to_msg()
        self.quality_pub.publish(msg)


def main():
    rclpy.init()
    node = GRConvNetGraspNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
