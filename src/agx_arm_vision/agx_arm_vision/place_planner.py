#!/usr/bin/env python3

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped
from image_geometry import PinholeCameraModel
from rclpy.node import Node
from scipy.spatial.transform import Rotation as R
from sensor_msgs.msg import CameraInfo, Image, PointCloud2, PointField
from std_msgs.msg import Bool, ColorRGBA, Header
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker


class PlacePlanner(Node):
    CAMERA_OPTICAL_FRAME = 'camera_color_optical_frame'
    MAP_ENABLE_TIMEOUT = 1.0

    def __init__(self):
        super().__init__('place_planner')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('camera_optical_frame', 'camera_color_optical_frame')
        self.declare_parameter(
            'depth_topic', '/camera/camera/aligned_depth_to_color/image_raw')
        self.declare_parameter(
            'info_topic', '/camera/camera/color/camera_info')
        self.declare_parameter('table_distance_threshold', 0.02)
        self.declare_parameter('occupied_height_threshold', 0.03)
        self.declare_parameter('min_clearance', 0.06)
        self.declare_parameter('max_range', 2.0)
        self.declare_parameter('min_range', 0.15)
        self.declare_parameter('reach_radius', 0.35)
        self.declare_parameter('process_period', 0.5)

        self.base_frame = self.get_parameter('base_frame').value
        self.camera_optical_frame = self.get_parameter(
            'camera_optical_frame').value
        self.table_dist = self.get_parameter('table_distance_threshold').value
        self.occupied_thresh = self.get_parameter(
            'occupied_height_threshold').value
        self.min_clearance = self.get_parameter('min_clearance').value
        self.max_range = self.get_parameter('max_range').value
        self.min_range = self.get_parameter('min_range').value
        self.reach_radius = self.get_parameter('reach_radius').value
        self.process_period = self.get_parameter('process_period').value

        self.bridge = CvBridge()
        self.model_cam = PinholeCameraModel()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.depth_img = None
        self.camera_info = None
        self.depth_stamp = None
        self._place_enabled = True
        self._last_place_msg_time = 0.0

        self.create_subscription(
            Image, self.get_parameter('depth_topic').value,
            self.depth_callback, 10)
        self.create_subscription(
            CameraInfo, self.get_parameter('info_topic').value,
            self.info_callback, 10)
        self.create_subscription(
            Bool, 'place_update_enable', self.place_update_cb, 10)

        self.place_pub = self.create_publisher(
            PoseStamped, 'place_pose', 10)
        self.cloud_pub = self.create_publisher(
            PointCloud2, 'place/points_filtered', 10)
        self.marker_pub = self.create_publisher(
            Marker, 'place_target_marker', 10)

        self.create_timer(self.process_period, self.process)
        self.get_logger().info('Place planner ready')

    def info_callback(self, msg):
        self.model_cam.fromCameraInfo(msg)
        self.camera_info = msg

    def depth_callback(self, msg):
        self.depth_img = self.bridge.imgmsg_to_cv2(msg, 'passthrough')
        self.depth_stamp = msg.header.stamp

    def place_update_cb(self, msg):
        self._place_enabled = msg.data
        self._last_place_msg_time = self.get_clock().now().nanoseconds * 1e-9

    def _cloud_gate(self):
        return self._place_enabled

    def _depth_meters(self, img):
        if img.dtype == np.uint16:
            return img.astype(np.float32) * 0.001
        return img.astype(np.float32)

    def _lookup_transform(self):
        try:
            return self.tf_buffer.lookup_transform(
                self.base_frame,
                self.camera_optical_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=1.0),
            )
        except TransformException:
            return None

    def _lookup_tcp_base(self):
        try:
            t = self.tf_buffer.lookup_transform(
                self.base_frame, 'tcp_link',
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=1.0),
            )
            return (t.transform.translation.x, t.transform.translation.y)
        except TransformException:
            return None

    def _ransac_plane(self, pts):
        if len(pts) < 300:
            return None
        sub = pts[::5] if len(pts) > 2000 else pts
        rng = np.random.default_rng()
        best_inl = 0
        best_n = None
        best_d = None
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

    def process(self):
        if self.depth_img is None or self.camera_info is None:
            return
        depth = self._depth_meters(self.depth_img)
        h, w = depth.shape
        fx = self.model_cam.fx()
        fy = self.model_cam.fy()
        fcx = self.model_cam.cx()
        fcy = self.model_cam.cy()

        transform = self._lookup_transform()
        if transform is None:
            return

        ds = 4
        uu, vv = np.meshgrid(
            np.arange(0, w, ds), np.arange(0, h, ds))
        z = depth[vv, uu]
        valid = (z > self.min_range) & (z < self.max_range) & np.isfinite(z)

        xs = (uu - fcx) * z / fx
        ys = (vv - fcy) * z / fy
        pts = np.stack([xs[valid], ys[valid], z[valid]], axis=1)
        plane = self._ransac_plane(pts)
        if plane is None:
            return
        n, d = plane

        heights = xs * n[0] + ys * n[1] + z * n[2] + d
        table_mask = (np.abs(heights) < self.table_dist) & valid
        occupied_mask = (heights > self.occupied_thresh) & valid

        # publish obstacle cloud (table + objects on table) for octomap
        if self._cloud_gate():
            cloud_keep = table_mask | occupied_mask
            self._publish_cloud_points(
                uu[cloud_keep], vv[cloud_keep], z[cloud_keep], transform)

        # find the empty spot farthest from obstacles and table edges,
        # but restricted to the arm's reachable area around the current TCP
        free = (table_mask & ~occupied_mask).astype(np.uint8)
        if self.reach_radius > 0.0:
            tcp_xy = self._lookup_tcp_base()
            if tcp_xy is not None:
                q = transform.transform.rotation
                mat = R.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
                tx = transform.transform.translation.x
                ty = transform.transform.translation.y
                bx = mat[0, 0] * xs + mat[0, 1] * ys + mat[0, 2] * z + tx
                by = mat[1, 0] * xs + mat[1, 1] * ys + mat[1, 2] * z + ty
                d2 = (bx - tcp_xy[0]) ** 2 + (by - tcp_xy[1]) ** 2
                reachable = (d2 <= self.reach_radius ** 2) & valid
                free &= reachable
        if free.sum() < 50:
            return
        dist = cv2.distanceTransform(free, cv2.DIST_L2, 3)
        max_d = float(dist.max())
        if not np.isfinite(max_d) or max_d <= 0:
            return
        v_idx, u_idx = np.unravel_index(np.argmax(dist), dist.shape)
        u_spot = int(u_idx * ds)
        v_spot = int(v_idx * ds)

        denom = (
            (u_spot - fcx) / fx * n[0]
            + (v_spot - fcy) / fy * n[1]
            + n[2])
        if abs(denom) < 1e-6:
            return
        z_spot = -d / denom
        if not np.isfinite(z_spot) or z_spot <= 0.05 or z_spot > self.max_range:
            return

        clearance_m = max_d * ds * z_spot / fx
        if clearance_m < self.min_clearance:
            return

        x_spot = (u_spot - fcx) * z_spot / fx
        y_spot = (v_spot - fcy) * z_spot / fy
        base_pt = self._cam_to_base(
            x_spot, y_spot, z_spot, transform)

        self._publish_place_pose(base_pt, transform)
        self._publish_marker(base_pt)
        self.get_logger().info(
            f'Place spot: ({base_pt[0]:.3f}, {base_pt[1]:.3f}, '
            f'{base_pt[2]:.3f}) clearance={clearance_m:.3f}m')

    def _cam_to_base(self, x, y, z, transform):
        q = transform.transform.rotation
        mat = R.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
        tx, ty, tz = (
            transform.transform.translation.x,
            transform.transform.translation.y,
            transform.transform.translation.z)
        p = mat @ np.array([x, y, z]) + np.array([tx, ty, tz])
        return (float(p[0]), float(p[1]), float(p[2]))

    def _publish_place_pose(self, base_pt, transform):
        pose = PoseStamped()
        pose.header.frame_id = self.base_frame
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = base_pt[0]
        pose.pose.position.y = base_pt[1]
        pose.pose.position.z = base_pt[2]
        pose.pose.orientation.w = 1.0
        self.place_pub.publish(pose)

    def _publish_marker(self, base_pt):
        marker = Marker()
        marker.header.frame_id = self.base_frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'place_target'
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = base_pt[0]
        marker.pose.position.y = base_pt[1]
        marker.pose.position.z = base_pt[2]
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.03
        marker.scale.y = 0.03
        marker.scale.z = 0.03
        marker.color = ColorRGBA(r=0.2, g=1.0, b=0.2, a=0.9)
        self.marker_pub.publish(marker)

    def _publish_cloud_points(self, uu, vv, z, transform):
        if len(z) == 0:
            return
        fx = self.model_cam.fx()
        fy = self.model_cam.fy()
        fcx = self.model_cam.cx()
        fcy = self.model_cam.cy()
        xs = (uu - fcx) * z / fx
        ys = (vv - fcy) * z / fy
        points = np.stack([xs, ys, z], axis=1)
        q = transform.transform.rotation
        mat = R.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
        tx, ty, tz = (
            transform.transform.translation.x,
            transform.transform.translation.y,
            transform.transform.translation.z)
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
        self.cloud_pub.publish(cloud)


def main():
    rclpy.init()
    node = PlacePlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
