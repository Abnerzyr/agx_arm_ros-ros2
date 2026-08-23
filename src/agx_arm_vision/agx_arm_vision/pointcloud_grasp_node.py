#!/usr/bin/env python3

import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped
from image_geometry import PinholeCameraModel
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image, JointState, PointCloud2, PointField
from std_msgs.msg import Header
from tf2_geometry_msgs import do_transform_pose_stamped
from tf2_ros import Buffer, TransformException, TransformListener


class PointCloudGraspNode(Node):
    def __init__(self):
        super().__init__('pointcloud_grasp_node')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('depth_topic',
                               '/camera/camera/aligned_depth_to_color/image_raw')
        self.declare_parameter('info_topic',
                               '/camera/camera/color/camera_info')
        self.declare_parameter('plane_distance', 0.015)
        self.declare_parameter('cluster_tolerance', 0.02)
        self.declare_parameter('min_cluster_size', 100)
        self.declare_parameter('ransac_samples', 500)
        self.declare_parameter('downsample_step', 2)
        self.declare_parameter('publish_static_joints', False)
        self.declare_parameter(
            'initial_joints',
            [-0.0259, -0.4025, -0.0575, 2.0, 0.0604, 0.0722, 0.9141])

        self.base_frame = self.get_parameter('base_frame').value
        self.plane_dist = self.get_parameter('plane_distance').value
        self.cluster_tol = self.get_parameter('cluster_tolerance').value
        self.min_cluster = self.get_parameter('min_cluster_size').value
        self.ransac_n = self.get_parameter('ransac_samples').value
        self.ds = self.get_parameter('downsample_step').value

        self.bridge = CvBridge()
        self.model = PinholeCameraModel()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.depth = None
        self.camera_info = None

        self.create_subscription(
            Image, self.get_parameter('depth_topic').value,
            self.depth_callback, 10)
        self.create_subscription(
            CameraInfo, self.get_parameter('info_topic').value,
            self.info_callback, 10)
        self.grasp_pub = self.create_publisher(
            PoseStamped, '/grasp_pose', 10)
        self.cloud_pub = self.create_publisher(
            PointCloud2, '/pointcloud_grasp/objects', 10)

        if self.get_parameter('publish_static_joints').value:
            self._start_static_joints()

        self.create_timer(0.2, self.process)
        self.get_logger().info('PointCloud grasp node ready')

    def info_callback(self, msg):
        self.model.fromCameraInfo(msg)
        self.camera_info = msg

    def depth_callback(self, msg):
        self.depth = self.bridge.imgmsg_to_cv2(msg, 'passthrough')

    def process(self):
        if hasattr(self, '_static_joint_pub'):
            self._static_joint_msg.header.stamp = (
                self.get_clock().now().to_msg())
            self._static_joint_pub.publish(self._static_joint_msg)
        if self.depth is None or self.camera_info is None:
            return
        cloud = self._depth_to_cloud(self.depth)
        if cloud is None or len(cloud) < self.ransac_n:
            return
        objects = self._remove_plane(cloud)
        if objects is None or len(objects) < self.min_cluster:
            return
        clusters = self._cluster(objects)
        if not clusters:
            return
        best = max(clusters, key=lambda c: len(c))
        if len(best) < self.min_cluster:
            return
        centroid = best.mean(axis=0)
        self._publish_cloud(best)
        self._publish(centroid)

    def _depth_to_cloud(self, depth):
        h, w = depth.shape[:2]
        fx = self.model.fx()
        fy = self.model.fy()
        cx = self.model.cx()
        cy = self.model.cy()
        u = np.arange(0, w, self.ds)
        v = np.arange(0, h, self.ds)
        uu, vv = np.meshgrid(u, v)
        z = depth[vv, uu].astype(np.float32)
        if depth.dtype == np.uint16:
            z *= 0.001
        valid = (z > 0.05) & (z < 2.0) & np.isfinite(z)
        z = z[valid]
        uu = uu[valid]
        vv = vv[valid]
        x = (uu - cx) * z / fx
        y = (vv - cy) * z / fy
        return np.stack([x, y, z], axis=1)

    def _remove_plane(self, points):
        if len(points) < self.ransac_n:
            return None
        best = (0, None)
        for _ in range(50):
            idx = np.random.choice(len(points), 3, replace=False)
            v1 = points[idx[1]] - points[idx[0]]
            v2 = points[idx[2]] - points[idx[0]]
            normal = np.cross(v1, v2)
            norm = np.linalg.norm(normal)
            if norm < 1e-6:
                continue
            normal /= norm
            dists = np.abs(np.dot(points - points[idx[0]], normal))
            inliers = np.sum(dists < self.plane_dist)
            if inliers > best[0]:
                best = (inliers, normal, points[idx[0]])
        if best[0] < self.ransac_n:
            return None
        _, normal, point = best
        dists = np.abs(np.dot(points - point, normal))
        return points[dists > self.plane_dist]

    def _cluster(self, points):
        if len(points) > 3000:
            idx = np.random.choice(len(points), 3000, replace=False)
            points = points[idx]
        labels = -np.ones(len(points), dtype=int)
        cluster_id = 0
        for i in range(len(points)):
            if labels[i] >= 0:
                continue
            seed = [i]
            labels[i] = cluster_id
            for s in seed:
                dists = np.linalg.norm(points - points[s], axis=1)
                neighbors = np.where(
                    (labels < 0) & (dists < self.cluster_tol))[0]
                for n in neighbors:
                    labels[n] = cluster_id
                    seed.append(n)
            cluster_id += 1
        clusters = []
        for cid in range(cluster_id):
            mask = labels == cid
            if mask.sum() >= self.min_cluster:
                clusters.append(points[mask])
        return clusters

    def _publish_cloud(self, points):
        try:
            t = self.tf_buffer.lookup_transform(
                self.base_frame,
                self.camera_info.header.frame_id,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=1.0),
            )
        except TransformException:
            return
        hdr = Header()
        hdr.frame_id = self.base_frame
        hdr.stamp = self.get_clock().now().to_msg()
        tx = t.transform.translation.x
        ty = t.transform.translation.y
        tz = t.transform.translation.z
        q = t.transform.rotation
        from scipy.spatial.transform import Rotation as R
        mat = R.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
        p = (mat @ points.T).T + np.array([tx, ty, tz])

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

    def _publish(self, centroid):
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
        pose = PoseStamped()
        pose.header.frame_id = self.camera_info.header.frame_id
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(centroid[0])
        pose.pose.position.y = float(centroid[1])
        pose.pose.position.z = float(centroid[2])
        pose.pose.orientation.w = 1.0
        base_pose = do_transform_pose_stamped(pose, transform)
        base_pose.pose.position.y = base_pose.pose.position.y
        base_pose.pose.orientation.w = 1.0
        base_pose.pose.orientation.x = 0.0
        base_pose.pose.orientation.y = 0.0
        base_pose.pose.orientation.z = 0.0
        self.grasp_pub.publish(base_pose)

    def _start_static_joints(self):
        self._static_joint_pub = self.create_publisher(
            JointState, '/joint_states', 10)
        names = ['joint1', 'joint2', 'joint3', 'joint4',
                 'joint5', 'joint6', 'joint7', 'gripper']
        values = self.get_parameter('initial_joints').value
        self._static_joint_msg = JointState()
        self._static_joint_msg.name = names
        self._static_joint_msg.position = (
            list(values) + [0.0])
        self.get_logger().info(
            f'Publishing static joint states: {values}')


def main():
    rclpy.init()
    node = PointCloudGraspNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
