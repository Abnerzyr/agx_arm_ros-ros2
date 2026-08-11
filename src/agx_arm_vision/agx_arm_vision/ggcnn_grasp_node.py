#!/usr/bin/env python3

import cv2
import math
import os

import numpy as np
import rclpy
import torch
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped
from image_geometry import PinholeCameraModel
from rclpy.node import Node
from scipy.spatial.transform import Rotation as R
from sensor_msgs.msg import CameraInfo, Image, PointCloud2, PointField
from std_msgs.msg import Header
from std_msgs.msg import Header
from tf2_geometry_msgs import do_transform_pose_stamped
from tf2_ros import Buffer, TransformException, TransformListener
from agx_arm_vision.models.ggcnn import GGCNN

class GGCNNGraspNode(Node):
    def __init__(self):
        super().__init__('ggcnn_grasp_node')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter(
            'depth_topic', '/camera/camera/aligned_depth_to_color/image_raw')
        self.declare_parameter(
            'info_topic', '/camera/camera/color/camera_info')
        self.declare_parameter('model_path', '')
        self.declare_parameter('quality_threshold', 0.3)

        self.base_frame = self.get_parameter('base_frame').value

        self.bridge = CvBridge()
        self.model_cam = PinholeCameraModel()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.depth_img = None
        self.camera_info = None

        self.model = self._load_model()
        self.get_logger().info('GG-CNN model loaded')

        self.create_subscription(
            Image, self.get_parameter('depth_topic').value,
            self.depth_callback, 10)
        self.create_subscription(
            CameraInfo, self.get_parameter('info_topic').value,
            self.info_callback, 10)
        self.grasp_pub = self.create_publisher(
            PoseStamped, '/grasp_pose', 10)
        self.cloud_pub = self.create_publisher(
            PointCloud2, '/ggcnn/points', 10)
        self.create_timer(0.5, self.process)
        self.get_logger().info('GG-CNN grasp node ready')

    def _load_model(self):
        model_path = self.get_parameter('model_path').value
        if model_path:
            state = torch.load(model_path, map_location='cpu')
        else:
            pkg_dir = os.path.dirname(os.path.abspath(__file__))
            weight_file = os.path.join(
                pkg_dir, 'models', 'ggcnn_epoch_23_cornell_statedict.pt')
            state = torch.load(weight_file, map_location='cpu')
        model = GGCNN()
        model.load_state_dict(state)
        model.eval()
        return model

    def info_callback(self, msg):
        self.model_cam.fromCameraInfo(msg)
        self.camera_info = msg

    def depth_callback(self, msg):
        self.depth_img = self.bridge.imgmsg_to_cv2(msg, 'passthrough')

    def _depth_meters(self, img):
        if img.dtype == np.uint16:
            return img.astype(np.float32) * 0.001
        return img.astype(np.float32)

    def process(self):
        if self.depth_img is None or self.camera_info is None:
            return
        depth = self._depth_meters(self.depth_img)
        self._publish_cloud(depth)
        h, w = depth.shape
        crop_size = min(h, w)
        start_h = (h - crop_size) // 2
        start_w = (w - crop_size) // 2
        depth_crop = depth[start_h:start_h + crop_size,
                           start_w:start_w + crop_size].copy()
        mask = (depth_crop == 0) | (~np.isfinite(depth_crop))
        depth_crop[mask] = 0.0
        if mask.any():
            depth_crop = cv2.inpaint(
                depth_crop, mask.astype(np.uint8), 3, cv2.INPAINT_TELEA)
        depth_input = cv2.resize(depth_crop, (300, 300))
        depth_input = depth_input - depth_input.mean()
        depth_input = depth_input / max(depth_input.std(), 1e-6)
        depth_input = np.clip(depth_input, -1.0, 1.0)
        tensor = torch.from_numpy(depth_input).float().unsqueeze(0).unsqueeze(0)
        with torch.no_grad():
            quality, cos_ang, sin_ang, width = self.model(tensor)
        quality_np = quality.squeeze().cpu().numpy()
        cos_np = cos_ang.squeeze().cpu().numpy()
        sin_np = sin_ang.squeeze().cpu().numpy()
        width_np = width.squeeze().cpu().numpy()
        best_pixel = np.unravel_index(np.argmax(quality_np), quality_np.shape)
        best_score = quality_np[best_pixel]
        if best_score < self.quality_threshold:
            return
        v, u = best_pixel
        angle = math.atan2(sin_np[v, u], cos_np[v, u]) / 2.0
        grasp_width = max(0.0, min(0.1, width_np[v, u]))
        scale = crop_size / 300
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
        x = (orig_u - cx) * z / fx
        y = (cy - orig_v) * z / fy
        self._publish(x, y, z, angle, grasp_width, best_score)

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
        tx = t.transform.translation.x
        ty = t.transform.translation.y
        tz = t.transform.translation.z
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


def main():
    rclpy.init()
    node = GGCNNGraspNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
