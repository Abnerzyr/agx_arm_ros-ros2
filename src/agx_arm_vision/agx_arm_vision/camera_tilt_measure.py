#!/usr/bin/env python3

import math

import numpy as np
import rclpy
from cv_bridge import CvBridge
from image_geometry import PinholeCameraModel
from rclpy.node import Node
from scipy.spatial.transform import Rotation as R
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformException, TransformListener


class TiltMeasureNode(Node):
    def __init__(self):
        super().__init__('tilt_measure')
        self.declare_parameter(
            'depth_topic', '/camera/camera/aligned_depth_to_color/image_raw')
        self.declare_parameter(
            'info_topic', '/camera/camera/color/camera_info')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('optical_frame', 'camera_color_optical_frame')
        self.declare_parameter('link_frame', 'camera_link')
        self.declare_parameter('frames', 30)
        self.declare_parameter('mount_rpy', [0.0, 0.0, -1.5708])

        self.frames = self.get_parameter('frames').value
        self.optical_frame = self.get_parameter('optical_frame').value
        self.link_frame = self.get_parameter('link_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.mount_rpy = self.get_parameter('mount_rpy').value

        self.bridge = CvBridge()
        self.model = PinholeCameraModel()
        self.depth_img = None
        self.normals = []
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self._sub_depth = self.create_subscription(
            Image, self.get_parameter('depth_topic').value,
            self.depth_cb, 10)
        self._sub_info = self.create_subscription(
            CameraInfo, self.get_parameter('info_topic').value,
            self.info_cb, 10)
        self._timer = self.create_timer(0.5, self.tick)
        self.get_logger().info(
            f'Tilt measure ready, collecting {self.frames} frames')

    def info_cb(self, msg):
        self.model.fromCameraInfo(msg)

    def depth_cb(self, msg):
        self.depth_img = self.bridge.imgmsg_to_cv2(msg, 'passthrough')

    def tick(self):
        if self.depth_img is None or self.model.fx() == 0.0:
            return
        d = self.depth_img.astype(np.float32)
        if self.depth_img.dtype == np.uint16:
            d *= 0.001
        h, w = d.shape
        ds = 4
        u = np.arange(0, w, ds)
        v = np.arange(0, h, ds)
        uu, vv = np.meshgrid(u, v)
        z = d[vv, uu]
        valid = (z > 0.05) & (z < 2.0) & np.isfinite(z)
        x = (uu - self.model.cx()) * z / self.model.fx()
        y = (vv - self.model.cy()) * z / self.model.fy()
        pts = np.stack([x[valid], y[valid], z[valid]], axis=1)
        n = self._ransac_plane(pts)
        if n is None:
            return
        self.normals.append(n)
        self.get_logger().info(
            f'frame {len(self.normals)}/{self.frames} collected')
        if len(self.normals) >= self.frames:
            self._finish()

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
        if best_inl < 0.3 * len(sub):
            return None
        if best_n[2] > 0:
            best_n = -best_n
        return best_n

    def _lookup_rot(self, target, source):
        t = self.tf_buffer.lookup_transform(
            target, source, rclpy.time.Time(),
            timeout=rclpy.duration.Duration(seconds=1.0))
        q = t.transform.rotation
        return R.from_quat([q.x, q.y, q.z, q.w]).as_matrix()

    def _finish(self):
        self._timer.cancel()
        n_meas = np.mean(self.normals, axis=0)
        n_meas /= np.linalg.norm(n_meas)
        try:
            r_opt = self._lookup_rot(self.base_frame, self.optical_frame)
            r_gb = self._lookup_rot(self.base_frame, 'gripper_base')
        except TransformException:
            self.get_logger().error('TF lookup failed')
            return
        n_pred = r_opt[2, :]
        cosang = float(np.clip(n_meas @ n_pred, -1.0, 1.0))
        ang = math.degrees(math.acos(cosang))
        self.get_logger().info(
            f'n_meas(cam) = ({n_meas[0]:.4f}, '
            f'{n_meas[1]:.4f}, {n_meas[2]:.4f})')
        self.get_logger().info(
            f'n_pred(cam) = ({n_pred[0]:.4f}, '
            f'{n_pred[1]:.4f}, {n_pred[2]:.4f})')
        self.get_logger().info(f'TILT ANGLE = {ang:.3f} deg')
        if ang < 0.2:
            self.get_logger().info('Tilt negligible; no correction needed')
            self._done()
            return
        n_base = r_opt @ n_meas
        axis_b = np.cross(n_base, [0.0, 0.0, 1.0])
        norm = np.linalg.norm(axis_b)
        if norm < 1e-9:
            c_base = R.identity()
        else:
            axis_b = axis_b / norm
            c_base = R.from_rotvec(
                axis_b * math.acos(float(np.clip(n_base[2], -1.0, 1.0))))
        c_gb = r_gb.T @ c_base.as_matrix() @ r_gb
        mount_old = R.from_euler(
            'xyz', [float(v) for v in self.mount_rpy])
        mount_new = R.from_matrix(c_gb @ mount_old.as_matrix())
        euler_new = mount_new.as_euler('xyz')
        self.get_logger().info(
            'Measured table normal in base: '
            f'({n_base[0]:.4f}, {n_base[1]:.4f}, {n_base[2]:.4f})')
        self.get_logger().info(
            'New camera_mount_joint rpy (paste into URDF): '
            f'roll={euler_new[0]:.5f} pitch={euler_new[1]:.5f} '
            f'yaw={euler_new[2]:.5f}')
        self._done()

    def _done(self):
        self.get_logger().info('Measurement finished')
        self.destroy_subscription(self._sub_depth)
        self.destroy_subscription(self._sub_info)
        self.destroy_node()


def main():
    rclpy.init()
    node = TiltMeasureNode()
    try:
        rclpy.spin(node)
    except Exception:
        pass
    rclpy.shutdown()


if __name__ == '__main__':
    main()
