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
from ultralytics import YOLO


class YoloGraspNode(Node):
    CAMERA_OPTICAL_FRAME = 'camera_color_optical_frame'

    def __init__(self):
        super().__init__('yolo_grasp_node')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter(
            'depth_topic', '/camera/camera/aligned_depth_to_color/image_raw')
        self.declare_parameter(
            'rgb_topic', '/camera/camera/color/image_raw')
        self.declare_parameter(
            'info_topic', '/camera/camera/color/camera_info')
        self.declare_parameter('confidence_threshold', 0.25)
        self.declare_parameter('grasp_quality_threshold', 0.3)
        self.declare_parameter('input_size', 224)

        self.base_frame = self.get_parameter('base_frame').value
        self.conf_threshold = self.get_parameter('confidence_threshold').value
        self.grasp_quality = self.get_parameter('grasp_quality_threshold').value
        self.input_size = self.get_parameter('input_size').value

        self.bridge = CvBridge()
        self.model_cam = PinholeCameraModel()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.depth_img = None
        self.rgb_img = None
        self.camera_info = None

        pkg_dir = os.path.dirname(os.path.abspath(__file__))
        yolo_path = os.path.join(pkg_dir, 'models', 'yolov8s-worldv2.pt')
        self.yolo = YOLO(yolo_path)
        self.get_logger().info('YOLOv8n loaded')

        self.grconv = self._load_grconv()
        self.get_logger().info('GR-ConvNet loaded')

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
        self.det_img_pub = self.create_publisher(
            Image, '/yolo/detections', 10)
        self.crop_img_pub = self.create_publisher(
            Image, '/yolo/crop_rgb', 10)
        self.quality_pub = self.create_publisher(
            Image, '/yolo/quality_map', 10)
        self.cloud_pub = self.create_publisher(
            PointCloud2, '/yolo/points', 10)

        self.create_timer(0.5, self.process)
        self.get_logger().info('YOLO+Grasp node ready')

    def _load_grconv(self):
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

        results = self.yolo(rgb, verbose=False)
        boxes = results[0].boxes
        if boxes is None or len(boxes) == 0:
            return

        det_img = results[0].plot()
        det_img = cv2.cvtColor(det_img, cv2.COLOR_RGB2BGR)
        self._publish_image(self.det_img_pub, det_img)

        best_box = None
        best_depth = float('inf')
        for box in boxes:
            conf = float(box.conf[0])
            if conf < self.conf_threshold:
                continue
            x1, y1, x2, y2 = map(int, box.xyxy[0])
            x1, x2 = max(0, x1), min(w - 1, x2)
            y1, y2 = max(0, y1), min(h - 1, y2)
            if x2 <= x1 or y2 <= y1:
                continue
            d = depth[y1:y2, x1:x2]
            valid = d[d > 0.05]
            if len(valid) < 50:
                continue
            med = np.median(valid)
            if med < best_depth:
                best_depth = med
                best_box = (x1, y1, x2, y2)

        if best_box is None:
            return
        x1, y1, x2, y2 = best_box

        cx = (x1 + x2) // 2
        cy = (y1 + y2) // 2
        bw = x2 - x1
        bh = y2 - y1
        margin = int(max(bw, bh) * 0.2)
        crop_x1 = max(0, x1 - margin)
        crop_y1 = max(0, y1 - margin)
        crop_x2 = min(w, x2 + margin)
        crop_y2 = min(h, y2 + margin)
        crop_side = max(crop_x2 - crop_x1, crop_y2 - crop_y1)
        ch = crop_y2 - crop_y1
        cw_real = crop_x2 - crop_x1
        patch_rgb = rgb[crop_y1:crop_y2, crop_x1:crop_x2]
        crop_rgb = cv2.copyMakeBorder(
            patch_rgb, 0, crop_side - ch, 0, crop_side - cw_real,
            cv2.BORDER_REPLICATE)
        patch_d = depth[crop_y1:crop_y2, crop_x1:crop_x2]
        crop_d = cv2.copyMakeBorder(
            patch_d, 0, crop_side - ch, 0, crop_side - cw_real,
            cv2.BORDER_REPLICATE)

        self._publish_image(self.crop_img_pub, crop_rgb)

        d_valid = (crop_d > 0.05) & np.isfinite(crop_d)
        if d_valid.sum() < 100:
            return
        mask = ~d_valid
        if mask.any():
            crop_d = cv2.inpaint(crop_d, mask.astype(np.uint8), 3, cv2.INPAINT_TELEA)

        d_resized = cv2.resize(crop_d, (self.input_size, self.input_size))
        r_resized = cv2.resize(crop_rgb, (self.input_size, self.input_size))

        d_norm = (d_resized - d_resized.mean()) / max(d_resized.std(), 1e-6)
        d_norm = np.clip(d_norm, -1.0, 1.0)
        r_norm = r_resized.astype(np.float32) / 255.0
        r_norm = (r_norm - np.array([0.485, 0.456, 0.406])) / np.array([0.229, 0.224, 0.225])
        r_norm = np.clip(r_norm, -3.0, 3.0)

        x_in = np.concatenate(
            [r_norm.transpose(2, 0, 1), d_norm[np.newaxis, :, :]], axis=0)
        tensor = torch.from_numpy(x_in).float().unsqueeze(0)
        with torch.no_grad():
            quality, cos_ang, sin_ang, width = self.grconv(tensor)

        quality_np = gaussian(quality.squeeze().cpu().numpy(), 2.0, preserve_range=True)
        self._publish_quality(quality_np)
        ang_np = (torch.atan2(sin_ang, cos_ang) / 2.0).squeeze().cpu().numpy()
        ang_np = gaussian(ang_np, 2.0, preserve_range=True)
        width_np = width.squeeze().cpu().numpy() * 150.0

        best_px = np.unravel_index(np.argmax(quality_np), quality_np.shape)
        best_score = quality_np[best_px]
        if best_score < self.grasp_quality:
            return

        v, u = best_px
        angle = ang_np[v, u]
        grasp_width = max(0.0, min(0.1, width_np[v, u] * 0.001))

        z_center = depth[cy, cx] if (0 <= cy < h and 0 <= cx < w) else best_depth
        if z_center <= 0.05 or z_center > 2.0:
            z_center = best_depth
        if z_center <= 0.05 or z_center > 2.0:
            return

        fx = self.model_cam.fx()
        fy = self.model_cam.fy()
        fcx = self.model_cam.cx()
        fcy = self.model_cam.cy()
        x_pos = (cx - fcx) * z_center / fx
        y_pos = (cy - fcy) * z_center / fy

        self._publish_cloud(depth)
        self._publish(x_pos, y_pos, z_center, angle, grasp_width, best_score)

    def _publish(self, x, y, z, angle, width, score):
        try:
            transform = self.tf_buffer.lookup_transform(
                self.base_frame,
                self.CAMERA_OPTICAL_FRAME,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=1.0),
            )
        except TransformException:
            return
        grasp_rot = R.from_euler('z', angle)
        pose = PoseStamped()
        pose.header.frame_id = self.CAMERA_OPTICAL_FRAME
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
        base_pose.pose.position.y = base_pose.pose.position.y
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
                self.CAMERA_OPTICAL_FRAME,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=1.0),
            )
        except TransformException:
            return
        h, w = depth.shape
        fx = self.model_cam.fx()
        fy = self.model_cam.fy()
        fcx = self.model_cam.cx()
        fcy = self.model_cam.cy()
        ds = 4
        u = np.arange(0, w, ds)
        v = np.arange(0, h, ds)
        uu, vv = np.meshgrid(u, v)
        z = depth[vv, uu]
        valid = (z > 0.05) & (z < 2.0) & np.isfinite(z)
        z = z[valid]
        uu = uu[valid]
        vv = vv[valid]
        xs = (uu - fcx) * z / fx
        ys = (vv - fcy) * z / fy
        points = np.stack([xs, ys, z], axis=1)
        q = t.transform.rotation
        mat = R.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
        tx, ty, tz = t.transform.translation.x, t.transform.translation.y, t.transform.translation.z
        p = (mat @ points.T).T + np.array([tx, ty, tz])

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
            q_norm = np.zeros_like(quality_np, dtype=np.uint8)
        else:
            q_norm = ((quality_np - q_min) / (q_max - q_min) * 255).astype(np.uint8)
        q_color = cv2.applyColorMap(q_norm, cv2.COLORMAP_JET)
        self._publish_image(self.quality_pub, q_color)


    def _publish_image(self, pub, img):
        msg = Image()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.height = img.shape[0]
        msg.width = img.shape[1]
        msg.encoding = 'bgr8'
        msg.step = img.shape[1] * 3
        msg.is_bigendian = False
        msg.data = img.tobytes()
        pub.publish(msg)


def main():
    rclpy.init()
    node = YoloGraspNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
