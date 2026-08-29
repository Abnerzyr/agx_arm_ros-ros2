#!/usr/bin/env python3

import math
import os

import cv2
import numpy as np
import rclpy
import torch

# Jetson Orin：torch 默认抢占全部 6 核，推理时 CPU 打满导致 RViz/相机/规划卡顿。
# 限制为 2 线程：CPU 让给系统，同时降低线程缓冲的内存峰值（推理几乎不受影响）。
torch.set_num_threads(2)
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped
from image_geometry import PinholeCameraModel
from rclpy.node import Node
from scipy.spatial.transform import Rotation as R
from sensor_msgs.msg import (
    CameraInfo, Image, PointCloud2, PointField)
from skimage.filters import gaussian
from std_msgs.msg import Bool, ColorRGBA, Header
from tf2_geometry_msgs import do_transform_pose_stamped
from tf2_ros import Buffer, TransformException, TransformListener
from ultralytics import YOLO
from visualization_msgs.msg import Marker

from agx_arm_vision.grasp_rl import GraspRLRefiner, build_patch

try:
    from aruco_opencv_msgs.msg import ArucoDetection as _ArucoMsg
except ImportError:
    _ArucoMsg = None


class YoloGraspNode(Node):
    CAMERA_OPTICAL_FRAME = 'camera_color_optical_frame'
    MAP_ENABLE_TIMEOUT = 1.0

    @staticmethod
    def _rl_data_dir():
        """RL data root: $GRASP_RL_DATA_DIR if set, else ~/.grasp_rl."""
        return os.environ.get(
            'GRASP_RL_DATA_DIR',
            os.path.join(os.path.expanduser('~'), '.grasp_rl'))

    def __init__(self):
        super().__init__('yolo_grasp_node')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('camera_optical_frame', 'camera_color_optical_frame')
        self.declare_parameter(
            'depth_topic', '/camera/camera/aligned_depth_to_color/image_raw')
        self.declare_parameter(
            'rgb_topic', '/camera/camera/color/image_raw')
        self.declare_parameter(
            'info_topic', '/camera/camera/color/camera_info')
        self.declare_parameter('confidence_threshold', 0.15)
        self.declare_parameter('grasp_quality_threshold', 0.3)
        self.declare_parameter('max_grasp_depth', 2.0)
        self.declare_parameter('input_size', 224)
        self.declare_parameter('box_padding', 0.005)
        self.declare_parameter('min_range', 0.15)
        self.declare_parameter('box_exclude_window', 2.0)
        self.declare_parameter('prefer_aruco_nearby', True)
        self.declare_parameter('aruco_timeout', 8.0)
        self.declare_parameter('aruco_exclude_px', 8)
        self.declare_parameter('lock_target_enable', True)
        self.declare_parameter('lock_target_frames', 2)
        self.declare_parameter('lock_target_tol', 0.03)
        self.declare_parameter('masked_depth_window', True)
        self.declare_parameter('rl_enable', False)
        self.declare_parameter('rl_patch_size', 32)
        self.declare_parameter('rl_pixel_step', 6.0)
        self.declare_parameter('rl_angle_step_deg', 10.0)
        self.declare_parameter('rl_depth_step', 0.015)
        self.declare_parameter('rl_epsilon_start', 0.3)
        self.declare_parameter('rl_epsilon_end', 0.02)
        self.declare_parameter('rl_epsilon_decay', 0.998)
        self.declare_parameter('rl_replay_capacity', 500)
        self.declare_parameter('rl_batch_size', 32)
        self.declare_parameter('rl_grad_steps', 4)
        self.declare_parameter('rl_lr', 0.001)
        self.declare_parameter('rl_checkpoint_interval', 10)
        self.declare_parameter(
            'rl_checkpoint_dir',
            os.path.join(self._rl_data_dir(), 'checkpoint'))
        self.declare_parameter(
            'rl_log_dir',
            os.path.join(self._rl_data_dir(), 'samples'))
        self.declare_parameter('rl_inflight_timeout', 180.0)
        self.declare_parameter('rl_reward_success', 1.0)
        self.declare_parameter('rl_reward_empty', 0.2)
        self.declare_parameter('rl_reward_fail', -1.0)
        self.declare_parameter('rl_stats_interval', 20)

        self.base_frame = self.get_parameter('base_frame').value
        self.camera_optical_frame = self.get_parameter(
            'camera_optical_frame').value
        self.conf_threshold = self.get_parameter('confidence_threshold').value
        self.grasp_quality = self.get_parameter('grasp_quality_threshold').value
        self.max_grasp_depth = float(
            self.get_parameter('max_grasp_depth').value)
        self.input_size = self.get_parameter('input_size').value
        self.box_padding = self.get_parameter('box_padding').value
        self.min_range = self.get_parameter('min_range').value
        self.box_exclude_window = self.get_parameter(
            'box_exclude_window').value
        self.masked_depth_window = self.get_parameter(
            'masked_depth_window').value
        self.prefer_aruco_nearby = bool(
            self.get_parameter('prefer_aruco_nearby').value)
        self.aruco_timeout = float(self.get_parameter('aruco_timeout').value)
        self.aruco_exclude_px = float(
            self.get_parameter('aruco_exclude_px').value)
        self.lock_target_enable = bool(
            self.get_parameter('lock_target_enable').value)
        self.lock_target_frames = int(
            self.get_parameter('lock_target_frames').value)
        self.lock_target_tol = float(
            self.get_parameter('lock_target_tol').value)
        self.rl_enable = self.get_parameter('rl_enable').value
        self.rl_patch_size = int(
            self.get_parameter('rl_patch_size').value)

        self.bridge = CvBridge()
        self.model_cam = PinholeCameraModel()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.depth_img = None
        self.rgb_img = None
        self.camera_info = None
        self.depth_stamp = None
        self._map_enabled = True
        self._last_map_msg_time = 0.0
        self._cloud_ok = True
        self._table_plane = None
        self._plane_logged = False
        self._fallback_logged = False
        self._last_grasp_level = None
        self._last_target_exclude = None
        self._last_target_exclude_time = 0.0
        self._aruco_pose = None
        self._aruco_time = 0.0
        self._locked_target = None
        self._lock_candidate = None
        self._lock_frames = 0

        pkg_dir = os.path.dirname(os.path.abspath(__file__))
        yolo_path = os.path.join(pkg_dir, 'models', 'yolov8s-worldv2.pt')
        self.yolo = YOLO(yolo_path)
        self.get_logger().info(
            f'YOLO loaded with default classes '
            f'({len(self.yolo.names)} classes)')

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
        self.create_subscription(
            Bool, 'map_update_enable', self.map_update_cb, 10)
        if _ArucoMsg is not None:
            self.create_subscription(
                _ArucoMsg, '/aruco_detections', self.aruco_cb, 10)

        self.grasp_pub = self.create_publisher(
            PoseStamped, 'grasp_pose', 10)
        self.det_img_pub = self.create_publisher(
            Image, 'yolo/detections', 10)
        self.crop_img_pub = self.create_publisher(
            Image, 'yolo/crop_rgb', 10)
        self.quality_pub = self.create_publisher(
            Image, 'yolo/quality_map', 10)
        self.cloud_pub = self.create_publisher(
            PointCloud2, 'yolo/points', 10)
        self.filtered_cloud_pub = self.create_publisher(
            PointCloud2, 'yolo/points_filtered', 10)
        self.target_box_pub = self.create_publisher(
            Marker, 'yolo/target_box', 10)

        self.create_timer(0.5, self.process)
        self._rl = None
        if self.rl_enable:
            self._rl = GraspRLRefiner(self, {
                'patch_size': self.get_parameter('rl_patch_size').value,
                'pixel_step': self.get_parameter('rl_pixel_step').value,
                'angle_step_deg': self.get_parameter(
                    'rl_angle_step_deg').value,
                'depth_step': self.get_parameter('rl_depth_step').value,
                'epsilon_start': self.get_parameter(
                    'rl_epsilon_start').value,
                'epsilon_end': self.get_parameter('rl_epsilon_end').value,
                'epsilon_decay': self.get_parameter(
                    'rl_epsilon_decay').value,
                'replay_capacity': self.get_parameter(
                    'rl_replay_capacity').value,
                'batch_size': self.get_parameter('rl_batch_size').value,
                'grad_steps': self.get_parameter('rl_grad_steps').value,
                'lr': self.get_parameter('rl_lr').value,
                'checkpoint_interval': self.get_parameter(
                    'rl_checkpoint_interval').value,
                'checkpoint_dir': self.get_parameter(
                    'rl_checkpoint_dir').value,
                'log_dir': self.get_parameter('rl_log_dir').value,
                'inflight_timeout': self.get_parameter(
                    'rl_inflight_timeout').value,
                'reward_success': self.get_parameter(
                    'rl_reward_success').value,
                'reward_empty': self.get_parameter('rl_reward_empty').value,
                'reward_fail': self.get_parameter('rl_reward_fail').value,
                'stats_interval': self.get_parameter(
                    'rl_stats_interval').value,
                'n_scalars': 4,
            })
            self.get_logger().info('[RL] grasp refiner enabled')
        else:
            self.get_logger().info('RL disabled (passthrough mode)')

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
        self.depth_stamp = msg.header.stamp

    def rgb_callback(self, msg):
        self.rgb_img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

    def map_update_cb(self, msg):
        self._map_enabled = msg.data
        self._last_map_msg_time = self.get_clock().now().nanoseconds * 1e-9

    def _cloud_gate(self):
        return self._map_enabled

    def _depth_meters(self, img):
        if img.dtype == np.uint16:
            return img.astype(np.float32) * 0.001
        return img.astype(np.float32)

    def aruco_cb(self, msg):
        now = self.get_clock().now().nanoseconds * 1e-9
        if not msg.markers:
            self._aruco_pose = None
            self._aruco_time = now
            return
        self._aruco_pose = np.array([
            msg.markers[0].pose.position.x,
            msg.markers[0].pose.position.y,
            msg.markers[0].pose.position.z], np.float64)
        self._aruco_time = now
        self.get_logger().info(
            f'Aruco det: {len(msg.markers)} marker(s)',
            throttle_duration_sec=5.0)

    def _aruco_pixel(self):
        if self._aruco_pose is None:
            return None
        now = self.get_clock().now().nanoseconds * 1e-9
        if now - self._aruco_time > self.aruco_timeout:
            return None
        x, y, z = self._aruco_pose
        if z <= 0.05:
            return None
        u = self.model_cam.fx() * x / z + self.model_cam.cx()
        v = self.model_cam.fy() * y / z + self.model_cam.cy()
        return (u, v)

    def _is_aruco_box(self, box, aruco_px):
        """框是否属于 aruco：包含 aruco 像素，或中心距 aruco 像素在排除半径内。"""
        x1, y1, x2, y2 = box
        ua, va = aruco_px
        if x1 <= ua <= x2 and y1 <= va <= y2:
            return True
        uc = (x1 + x2) / 2.0
        vc = (y1 + y2) / 2.0
        return math.hypot(uc - ua, vc - va) <= self.aruco_exclude_px

    def _apply_target_lock(self, x, y, z, angle,
                           box_cx, box_cy, box_cz,
                           box_sx, box_sy, box_sz, box_quat):
        """目标冻结：抓取点连续 lock_target_frames 帧稳定后，锁定并持续发布
        同一目标（位置+朝向+框），直到目标移动超过 lock_target_tol 才重锁。
        返回冻结后的完整发布参数。"""
        if not self.lock_target_enable:
            return (x, y, z, angle,
                    box_cx, box_cy, box_cz,
                    box_sx, box_sy, box_sz, box_quat)
        cur = (x, y, z)
        if self._locked_target is not None:
            lt = self._locked_target
            if (abs(x - lt[0]) < self.lock_target_tol
                    and abs(y - lt[1]) < self.lock_target_tol
                    and abs(z - lt[2]) < self.lock_target_tol):
                return lt
            self._locked_target = None
            self._lock_candidate = cur
            self._lock_frames = 1
            self.get_logger().info('Target lock released (target changed)')
        elif (self._lock_candidate is not None
                and abs(x - self._lock_candidate[0]) < self.lock_target_tol
                and abs(y - self._lock_candidate[1]) < self.lock_target_tol
                and abs(z - self._lock_candidate[2]) < self.lock_target_tol):
            self._lock_frames += 1
            if self._lock_frames >= self.lock_target_frames:
                self._locked_target = (
                    x, y, z, angle,
                    box_cx, box_cy, box_cz,
                    box_sx, box_sy, box_sz, box_quat)
                self._lock_candidate = None
                self.get_logger().info(
                    f'Target locked at ({x:.3f},{y:.3f},{z:.3f})')
        else:
            self._lock_candidate = cur
            self._lock_frames = 1
        return (x, y, z, angle,
                box_cx, box_cy, box_cz,
                box_sx, box_sy, box_sz, box_quat)

    def _valid_boxes(self, boxes, depth, w, h):
        out = []
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
            out.append((x1, y1, x2, y2))
        return out

    def process(self):
        if (self.depth_img is None or self.rgb_img is None
                or self.camera_info is None):
            return
        self._cloud_ok = self._cloud_gate()
        depth = self._depth_meters(self.depth_img)
        rgb = self.rgb_img
        h, w = depth.shape

        results = self.yolo(rgb, verbose=False)
        boxes = results[0].boxes
        if boxes is None or len(boxes) == 0:
            self._publish_cloud(depth)
            return

        det_img = results[0].plot()
        det_img = cv2.cvtColor(det_img, cv2.COLOR_RGB2BGR)
        self._publish_image(self.det_img_pub, det_img)

        valid_boxes = self._valid_boxes(boxes, depth, w, h)

        aruco_px = None
        if self.prefer_aruco_nearby:
            aruco_px = self._aruco_pixel()

        best_box = None
        best_depth = float('inf')
        if aruco_px is not None:
            candidates = [
                b for b in valid_boxes if not self._is_aruco_box(b, aruco_px)]
            ua, va = aruco_px
            best_dist = None
            for (x1, y1, x2, y2) in candidates:
                uc = (x1 + x2) / 2.0
                vc = (y1 + y2) / 2.0
                dist = math.hypot(uc - ua, vc - va)
                if best_dist is None or dist < best_dist:
                    best_dist = dist
                    best_box = (x1, y1, x2, y2)
                    d = depth[y1:y2, x1:x2]
                    best_depth = np.median(d[d > 0.05])
            if best_box is not None:
                self.get_logger().info(
                    f'Aruco-aware target: px=({ua:.0f},{va:.0f}) '
                    f'box=({best_box[0]},{best_box[1]},{best_box[2]},'
                    f'{best_box[3]})', throttle_duration_sec=5.0)

        if best_box is None:
            if aruco_px is not None:
                candidates = [
                    b for b in valid_boxes
                    if not self._is_aruco_box(b, aruco_px)]
                if not candidates:
                    self.get_logger().info(
                        'Aruco present but no non-aruco box; '
                        'no grasp target this frame',
                        throttle_duration_sec=5.0)
            else:
                candidates = valid_boxes
            for (x1, y1, x2, y2) in candidates:
                d = depth[y1:y2, x1:x2]
                med = np.median(d[d > 0.05])
                if med < best_depth:
                    best_depth = med
                    best_box = (x1, y1, x2, y2)

        if best_box is None:
            now = self.get_clock().now().nanoseconds * 1e-9
            if (self._last_target_exclude is not None
                    and now - self._last_target_exclude_time
                    <= self.box_exclude_window):
                self._publish_cloud(depth, exclude_box=self._last_target_exclude)
            else:
                self._publish_cloud(depth)
            return
        x1, y1, x2, y2 = best_box

        cx = (x1 + x2) // 2
        cy = (y1 + y2) // 2
        bw = x2 - x1
        bh = y2 - y1
        margin_px = max(
            1, int(0.03 * self.model_cam.fx() / max(best_depth, 0.1)))
        exclude_box = (
            max(0, x1 - margin_px), max(0, y1 - margin_px),
            min(w - 1, x2 + margin_px), min(h - 1, y2 + margin_px))
        self._last_target_exclude = exclude_box
        self._last_target_exclude_time = (
            self.get_clock().now().nanoseconds * 1e-9)
        self._publish_cloud(depth, exclude_box=exclude_box)
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
        sel_valid = np.zeros((crop_side, crop_side), bool)
        by1, by2 = y1 - crop_y1, y2 - crop_y1
        bx1, bx2 = x1 - crop_x1, x2 - crop_x1
        sel_valid[by1:by2, bx1:bx2] = d_valid[by1:by2, bx1:bx2]
        fx = self.model_cam.fx()
        fy = self.model_cam.fy()
        fcx = self.model_cam.cx()
        fcy = self.model_cam.cy()
        plane = self._table_plane_from_frame(depth)
        if plane is not None:
            n, d = plane
            uu_c, vv_c = np.meshgrid(
                np.arange(crop_x1, crop_x2), np.arange(crop_y1, crop_y2))
            z_c = depth[crop_y1:crop_y2, crop_x1:crop_x2]
            ok_c = (z_c > 0.05) & (z_c < 2.0) & np.isfinite(z_c)
            xs_c = (uu_c - fcx) * z_c / fx
            ys_c = (vv_c - fcy) * z_c / fy
            dist_c = np.abs(xs_c * n[0] + ys_c * n[1] + z_c * n[2] + d)
            near = np.zeros((crop_side, crop_side), bool)
            near[:ch, :cw_real] = ok_c & (dist_c < 0.02)
            sel_valid &= ~near
        dist_px = cv2.distanceTransform(
            sel_valid.astype(np.uint8), cv2.DIST_L2, 3)
        thr_px = np.where(crop_d > 0.05, 0.01 * fx / crop_d, 0.0)
        sel_valid &= dist_px >= thr_px
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

        sel_small = cv2.resize(
            sel_valid.astype(np.uint8), (self.input_size, self.input_size),
            interpolation=cv2.INTER_NEAREST).astype(bool)
        quality_masked = np.where(sel_small, quality_np, -np.inf)
        best_px = np.unravel_index(
            np.argmax(quality_masked), quality_masked.shape)
        best_score = quality_masked[best_px]
        grasp_level = 0
        if not np.isfinite(best_score) or best_score < self.grasp_quality:
            relaxed = np.zeros((crop_side, crop_side), bool)
            relaxed[by1:by2, bx1:bx2] = d_valid[by1:by2, bx1:bx2]
            relaxed_small = cv2.resize(
                relaxed.astype(np.uint8),
                (self.input_size, self.input_size),
                interpolation=cv2.INTER_NEAREST).astype(bool)
            q1 = np.where(relaxed_small, quality_np, -np.inf)
            p1 = np.unravel_index(np.argmax(q1), q1.shape)
            s1 = q1[p1]
            if np.isfinite(s1):
                best_px, best_score, grasp_level = p1, s1, 1
        if not np.isfinite(best_score):
            p2 = np.unravel_index(np.argmax(quality_np), quality_np.shape)
            s2 = quality_np[p2]
            if np.isfinite(s2):
                best_px, best_score, grasp_level = p2, s2, 2
        if not np.isfinite(best_score):
            grasp_level = 3
            best_score = 0.0
        if grasp_level != self._last_grasp_level:
            self._last_grasp_level = grasp_level
            self.get_logger().info(
                f'Grasp point level {grasp_level} '
                f'(score={best_score:.2f})')

        if grasp_level == 3:
            u_orig = float(cx)
            v_orig = float(cy)
            angle = 0.0
            grasp_width = 0.0
        else:
            v, u = best_px
            angle = ang_np[v, u]
            grasp_width = max(0.0, min(0.1, width_np[v, u] * 0.001))
            scale = crop_side / self.input_size
            u_orig = crop_x1 + u * scale
            v_orig = crop_y1 + v * scale

        box_d = depth[y1:y2, x1:x2]
        box_valid = box_d[(box_d > 0.05) & np.isfinite(box_d)]
        if len(box_valid) > 0:
            d_min = float(np.min(box_valid))
            d_max = float(np.max(box_valid))
            d_p40 = float(np.percentile(box_valid, 40))
        else:
            d_min = d_max = d_p40 = float(best_depth)
        box_z = (d_min + d_p40) / 2.0
        if box_z <= 0.05 or box_z > 2.0:
            box_z = float(best_depth)
        if box_z <= 0.05 or box_z > 2.0:
            return
        box_fit = self._object_box_from_points(depth, x1, y1, x2, y2, plane)
        if box_fit is None:
            box_zc = (d_min + d_max) / 2.0
            box_cx = (cx - fcx) * box_zc / fx
            box_cy = (cy - fcy) * box_zc / fy
            box_cz = box_zc
            box_sx = min(bw * box_zc / fx, 0.3) + self.box_padding
            box_sy = min(bh * box_zc / fy, 0.3) + self.box_padding
            box_sz = min(max(d_max - d_min, 0.02), 0.2) + self.box_padding
            box_quat = self._table_box_quat(plane)
            self.get_logger().info(
                f'Box (fallback): d_min={d_min:.3f} d_p40={d_p40:.3f} '
                f'd_max={d_max:.3f} box_z={box_z:.3f} '
                f'box=({box_sx:.3f}x{box_sy:.3f}x{box_sz:.3f})')
        else:
            (box_cx, box_cy, box_cz), (box_sx, box_sy, box_sz), box_quat = box_fit
            self.get_logger().info(
                f'Box (world): center=({box_cx:.3f},{box_cy:.3f},'
                f'{box_cz:.3f}) size=({box_sx:.3f}x{box_sy:.3f}x{box_sz:.3f})')

        u_orig = min(max(u_orig, 0.0), float(w - 1))
        v_orig = min(max(v_orig, 0.0), float(h - 1))
        ui = int(round(u_orig))
        vi = int(round(v_orig))
        z_g = self._grasp_z_at(
            ui, vi, depth, sel_valid, crop_x1, crop_y1, crop_side, box_z)
        if self._rl is not None:
            patch = build_patch(depth, rgb, u_orig, v_orig,
                                self.rl_patch_size, z_g)
            scalars = np.array([
                z_g / 1.0,
                angle / math.pi,
                min(max(best_score, -1.0), 1.0),
                grasp_level / 3.0,
            ], np.float32)
            action = self._rl.observe(patch, scalars)
            u_orig, v_orig, angle, dz = self._rl.apply_action(
                action, u_orig, v_orig, angle, w, h)
            u_orig = min(max(u_orig, 0.0), float(w - 1))
            v_orig = min(max(v_orig, 0.0), float(h - 1))
            z_rl = self._grasp_z_at(
                int(round(u_orig)), int(round(v_orig)),
                depth, sel_valid, crop_x1, crop_y1, crop_side, box_z)
            z_g = min(max(z_rl + dz, 0.05), 2.0)
        x_g = (u_orig - fcx) * z_g / fx
        y_g = (v_orig - fcy) * z_g / fy

        (x_g, y_g, z_g, angle,
         box_cx, box_cy, box_cz,
         box_sx, box_sy, box_sz, box_quat) = self._apply_target_lock(
            x_g, y_g, z_g, angle,
            box_cx, box_cy, box_cz,
            box_sx, box_sy, box_sz, box_quat)

        self._publish(
            x_g, y_g, z_g, angle, grasp_width, best_score,
            box_center=(box_cx, box_cy, box_cz),
            box_scale=(box_sx, box_sy, box_sz),
            box_orientation=box_quat)

    def _grasp_z_at(self, ui, vi, depth, sel_valid, crop_x1, crop_y1,
                    crop_side, box_z):
        """Depth at a grasp pixel, constrained to the object-interior mask.

        Same semantics as the original 15x15 window logic; the mask keeps
        background depth out when the pixel sits near the object edge.
        """
        h, w = depth.shape
        r0 = max(0, vi - 7)
        r1 = min(h, vi + 8)
        c0 = max(0, ui - 7)
        c1 = min(w, ui + 8)
        win = depth[r0:r1, c0:c1]
        valid_mask = (win > 0.05) & np.isfinite(win)
        if self.masked_depth_window:
            mask = np.zeros((r1 - r0, c1 - c0), dtype=bool)
            cr0 = max(r0 - crop_y1, 0)
            cr1 = min(r1 - crop_y1, crop_side)
            cc0 = max(c0 - crop_x1, 0)
            cc1 = min(c1 - crop_x1, crop_side)
            if cr1 > cr0 and cc1 > cc0:
                mask[cr0 - (r0 - crop_y1): cr1 - (r0 - crop_y1),
                     cc0 - (c0 - crop_x1): cc1 - (c0 - crop_x1)] = \
                    sel_valid[cr0:cr1, cc0:cc1]
            if mask.any() and (valid_mask & mask).sum() < 10 \
                    and valid_mask.sum() >= 10:
                self.get_logger().warn(
                    'Masked depth window empty (obj mask excludes grasp '
                    'pixel); falling back to unmasked depth',
                    throttle_duration_sec=10.0)
            else:
                valid_mask = valid_mask & mask
        win_valid = win[valid_mask]
        if len(win_valid) >= 10:
            zg = float(np.percentile(win_valid, 40))
            if zg > self.max_grasp_depth:
                return box_z
            return zg
        return box_z

    def _publish(self, x, y, z, angle, width, score,
                 box_center=None, box_scale=None, box_orientation=None):
        if not self._cloud_ok:
            return
        try:
            transform = self.tf_buffer.lookup_transform(
                self.base_frame,
                self.camera_optical_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=1.0),
            )
        except TransformException:
            self.get_logger().error(
                'TF lookup failed; grasp pose not published',
                throttle_duration_sec=5.0)
            return
        grasp_rot = R.from_euler('z', angle)
        pose = PoseStamped()
        pose.header.frame_id = self.camera_optical_frame
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

        if box_center is not None and box_scale is not None:
            box_pose = PoseStamped()
            box_pose.header.frame_id = self.camera_optical_frame
            box_pose.header.stamp = pose.header.stamp
            box_pose.pose.position.x = float(box_center[0])
            box_pose.pose.position.y = float(box_center[1])
            box_pose.pose.position.z = float(box_center[2])
            if box_orientation is not None:
                box_pose.pose.orientation.x = float(box_orientation[0])
                box_pose.pose.orientation.y = float(box_orientation[1])
                box_pose.pose.orientation.z = float(box_orientation[2])
                box_pose.pose.orientation.w = float(box_orientation[3])
            else:
                box_pose.pose.orientation.w = 1.0
            base_box = do_transform_pose_stamped(box_pose, transform)
            marker = Marker()
            marker.header = base_box.header
            marker.ns = 'grasp_target_box'
            marker.id = 0
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            marker.pose = base_box.pose
            marker.scale.x = float(box_scale[0])
            marker.scale.y = float(box_scale[1])
            marker.scale.z = float(box_scale[2])
            marker.color = ColorRGBA(r=1.0, g=0.8, b=0.0, a=0.5)
            self.target_box_pub.publish(marker)

        self.grasp_pub.publish(base_pose)
        self.get_logger().info(
            f'Grasp: ({base_pose.pose.position.x:.3f}, '
            f'{base_pose.pose.position.y:.3f}, '
            f'{base_pose.pose.position.z:.3f}) '
            f'angle={math.degrees(angle):.1f}° '
            f'width={width:.3f}m score={score:.2f}')

    def _publish_cloud(self, depth, exclude_box=None):
        if not self._cloud_ok:
            return
        try:
            t = self.tf_buffer.lookup_transform(
                self.base_frame,
                self.camera_optical_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=1.0),
            )
        except TransformException:
            return
        h, w = depth.shape
        ds = 4
        u = np.arange(0, w, ds)
        v = np.arange(0, h, ds)
        uu, vv = np.meshgrid(u, v)
        z = depth[vv, uu]
        valid = (z > self.min_range) & (z < 2.0) & np.isfinite(z)

        self._publish_points(
            self.cloud_pub, uu[valid], vv[valid], z[valid], t)

        if exclude_box is not None:
            ex1, ey1, ex2, ey2 = exclude_box
            inside = (uu >= ex1) & (uu <= ex2) & (vv >= ey1) & (vv <= ey2)
            keep = valid & ~inside

            fx = self.model_cam.fx()
            fy = self.model_cam.fy()
            fcx = self.model_cam.cx()
            fcy = self.model_cam.cy()
            xs = (uu - fcx) * z / fx
            ys = (vv - fcy) * z / fy

            rot = R.from_quat([
                t.transform.rotation.x,
                t.transform.rotation.y,
                t.transform.rotation.z,
                t.transform.rotation.w,
            ]).as_matrix()
            n_cam = rot[2, :]

            plane = self._get_table_plane(
                n_cam,
                xs[inside & valid], ys[inside & valid], z[inside & valid])

            if plane is None:
                if not self._fallback_logged:
                    self.get_logger().warning(
                        'Table plane rejected; using full exclusion')
                    self._fallback_logged = True
                self._publish_points(
                    self.filtered_cloud_pub,
                    uu[keep], vv[keep], z[keep], t)
                return

            n, d = plane
            if not self._plane_logged:
                n_base = rot @ n
                self.get_logger().info(
                    'Table plane locked: '
                    f'n_base=({n_base[0]:.3f},{n_base[1]:.3f},'
                    f'{n_base[2]:.3f}) d={d:.3f}')
                self._plane_logged = True
            u_in = uu[inside]
            v_in = vv[inside]
            denom = (u_in - fcx) / fx * n[0] + (v_in - fcy) / fy * n[1] + n[2]
            z_plane = -d / denom
            ok = np.isfinite(z_plane) & (z_plane > 0.05) & (z_plane < 2.0)

            self._publish_points(
                self.filtered_cloud_pub,
                np.concatenate([uu[keep], u_in[ok]]),
                np.concatenate([vv[keep], v_in[ok]]),
                np.concatenate([z[keep], z_plane[ok]]),
                t)

    def _ransac_plane(self, pts):
        if len(pts) < 300:
            return None
        sub = pts[::5] if len(pts) > 2000 else pts
        rng = np.random.default_rng()
        best_inl = 0
        best_n = None
        for _ in range(60):
            i3 = rng.choice(len(sub), 3, replace=False)
            p0, p1, p2 = sub[i3]
            n = np.cross(p1 - p0, p2 - p0)
            norm = np.linalg.norm(n)
            if norm < 1e-9:
                continue
            n = n / norm
            d = -float(n @ p0)
            inl = int(np.count_nonzero(np.abs(sub @ n + d) < 0.012))
            if inl > best_inl:
                best_inl = inl
                best_n = n
                best_d = d
        if best_n is None or best_inl < 0.3 * len(sub):
            return None
        return best_n, best_d

    def _table_box_axes(self, plane=None):
        """Orthonormal table-aligned axes derived from the RANSAC plane.

        The height axis follows the table normal (gravity), so the fitted
        box lines up with the real object instead of the camera axes.
        Returns None if the base->camera TF is unavailable.
        """
        try:
            transform = self.tf_buffer.lookup_transform(
                self.base_frame,
                self.camera_optical_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=1.0),
            )
        except TransformException:
            return None
        q = transform.transform.rotation
        rot_cb = R.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
        rot_bc = rot_cb.T
        trans = np.array([
            transform.transform.translation.x,
            transform.transform.translation.y,
            transform.transform.translation.z])
        if plane is not None:
            n_cam = np.asarray(plane[0], dtype=float)
            if np.linalg.norm(n_cam) < 1e-9:
                n_base = np.array([0.0, 0.0, 1.0])
                d_base = None
            else:
                n_cam = n_cam / np.linalg.norm(n_cam)
                n_base = rot_cb @ n_cam
                d_base = float(plane[1]) - float(n_base @ trans)
        else:
            n_base = np.array([0.0, 0.0, 1.0])
            d_base = None
        tmp = (np.array([0.0, 1.0, 0.0])
               if abs(n_base[0]) > 0.9 else np.array([1.0, 0.0, 0.0]))
        e1_base = np.cross(n_base, tmp)
        e1_base = e1_base / np.linalg.norm(e1_base)
        e2_base = np.cross(n_base, e1_base)
        return {
            'rot_cb': rot_cb,
            'rot_bc': rot_bc,
            'trans': trans,
            'n_base': n_base,
            'd_base': d_base,
            'e1_base': e1_base,
            'e2_base': e2_base,
        }

    def _table_box_quat(self, plane=None):
        """Camera-frame quaternion of a table-aligned box (identity fallback)."""
        axes = self._table_box_axes(plane)
        if axes is None:
            return (0.0, 0.0, 0.0, 1.0)
        rot_base = np.column_stack(
            [axes['e1_base'], axes['e2_base'], axes['n_base']])
        rot_cam = axes['rot_bc'] @ rot_base
        quat = R.from_matrix(rot_cam).as_quat()
        return (float(quat[0]), float(quat[1]),
                float(quat[2]), float(quat[3]))

    def _object_box_from_points(self, depth, x1, y1, x2, y2, plane=None):
        if plane is None:
            plane = self._table_plane_from_frame(depth)
        axes = self._table_box_axes(plane)
        if axes is None:
            return None
        fx = self.model_cam.fx()
        fy = self.model_cam.fy()
        fcx = self.model_cam.cx()
        fcy = self.model_cam.cy()
        ds = 2
        uu, vv = np.meshgrid(
            np.arange(x1, x2, ds), np.arange(y1, y2, ds))
        z = depth[vv, uu]
        valid = (z > 0.05) & (z < 2.0) & np.isfinite(z)
        if valid.sum() < 50:
            return None
        xs = (uu - fcx) * z / fx
        ys = (vv - fcy) * z / fy
        pts_cam = np.stack([xs[valid], ys[valid], z[valid]], axis=1)
        pts_all = (axes['rot_cb'] @ pts_cam.T).T + axes['trans']
        pts = pts_all
        d_base = axes['d_base']
        if d_base is not None:
            n_base = axes['n_base']
            keep = np.abs(pts @ n_base + d_base) > 0.015
            pts = pts[keep]
            if len(pts) < 20:
                return None
        n_base = axes['n_base']
        e1_base = axes['e1_base']
        e2_base = axes['e2_base']
        px = pts @ e1_base
        py = pts @ e2_base
        h = pts @ n_base
        px_lo, px_hi = np.percentile(px, 2), np.percentile(px, 98)
        py_lo, py_hi = np.percentile(py, 2), np.percentile(py, 98)
        h_hi = float(np.percentile(h, 98))
        p_horiz = ((px_lo + px_hi) / 2.0) * e1_base + \
            ((py_lo + py_hi) / 2.0) * e2_base
        h_lo = float(np.percentile(h, 2))
        if d_base is not None:
            denom = float(n_base @ n_base)
            if abs(denom) > 1e-6:
                h_lo = float(-(n_base @ p_horiz + d_base) / denom)
        # Height floor from the box's nearest-depth points. When the object's
        # upper-surface depth is sparse, h_hi collapses toward the table and
        # the box becomes a flat slab; the nearest (front/top) band of the box
        # points still recovers the true object height.
        h_floor = 0.0
        if pts_cam.shape[0] > 0:
            d_min = float(np.min(pts_cam[:, 2]))
            near = pts_cam[:, 2] <= (d_min + 0.02)
            if near.sum() > 0:
                h_near = pts_all[near] @ n_base
                z_top = float(np.max(h_near))
                h_floor = min(max(0.0, z_top - h_lo), 0.2)
        box_dims = (
            float(px_hi - px_lo) + self.box_padding,
            float(py_hi - py_lo) + self.box_padding,
            max(float(h_hi - h_lo), h_floor, 0.01) + self.box_padding,
        )
        box_center_base = p_horiz + (float(h_lo + h_hi) / 2.0) * n_base
        box_center = axes['rot_bc'] @ (box_center_base - axes['trans'])
        if (not np.all(np.isfinite(box_dims))
                or max(box_dims) > 0.5
                or not np.all(np.isfinite(box_center))):
            return None
        rot_base = np.column_stack([e1_base, e2_base, n_base])
        rot_cam = axes['rot_bc'] @ rot_base
        quat = R.from_matrix(rot_cam).as_quat()
        return (tuple(float(c) for c in box_center),
                box_dims,
                tuple(float(qq) for qq in quat))

    def _table_plane_from_frame(self, depth):
        h, w = depth.shape
        fx = self.model_cam.fx()
        fy = self.model_cam.fy()
        fcx = self.model_cam.cx()
        fcy = self.model_cam.cy()
        ds = 8
        uu, vv = np.meshgrid(
            np.arange(0, w, ds), np.arange(0, h, ds))
        z = depth[vv, uu]
        valid = (z > self.min_range) & (z < 2.0) & np.isfinite(z)
        if valid.sum() < 300:
            return None
        xs = (uu - fcx) * z / fx
        ys = (vv - fcy) * z / fy
        pts = np.stack([xs[valid], ys[valid], z[valid]], axis=1)
        return self._ransac_plane(pts)

    def _get_table_plane(self, n, hx, hy, hz):
        if self._table_plane is not None:
            n0, d = self._table_plane
            if len(hx) >= 20:
                heights = hx * n0[0] + hy * n0[1] + hz * n0[2]
                low = float(np.percentile(heights, 5))
                if abs(low + d) <= 0.02:
                    return n0, d
            self._table_plane = None
        plane = self._fit_table_height(n, hx, hy, hz)
        if plane is not None:
            self._table_plane = plane
        return plane

    def _fit_table_height(self, n, hx, hy, hz):
        if len(hx) < 20:
            return None
        heights = hx * n[0] + hy * n[1] + hz * n[2]
        low = float(np.percentile(heights, 5))
        if not np.isfinite(low) or not (0.05 < abs(low) < 2.0):
            return None
        return n, -low

    def _publish_points(self, pub, uu, vv, z, t):
        fx = self.model_cam.fx()
        fy = self.model_cam.fy()
        fcx = self.model_cam.cx()
        fcy = self.model_cam.cy()
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
        hdr.stamp = (
            self.depth_stamp
            if self.depth_stamp is not None
            else self.get_clock().now().to_msg())
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
        pub.publish(cloud)

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

    def destroy_node(self):
        if self._rl is not None:
            self._rl.save_checkpoint()
        super().destroy_node()


def main():
    rclpy.init()
    node = YoloGraspNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
