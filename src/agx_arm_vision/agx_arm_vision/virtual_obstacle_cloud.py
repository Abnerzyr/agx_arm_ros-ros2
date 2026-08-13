#!/usr/bin/env python3

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header


class VirtualObstacleCloud(Node):
    def __init__(self):
        super().__init__('virtual_obstacle_cloud')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('rate', 2.0)
        self.declare_parameter('table_z', 0.25)
        self.declare_parameter('table_half', 0.35)
        self.declare_parameter('table_step', 0.02)
        self.declare_parameter('obstacle_x', 0.35)
        self.declare_parameter('obstacle_y', 0.0)
        self.declare_parameter('obstacle_z', 0.30)
        self.declare_parameter('obstacle_size', 0.10)

        self.base_frame = self.get_parameter('base_frame').value
        self.rate = self.get_parameter('rate').value
        self.table_z = self.get_parameter('table_z').value
        self.table_half = self.get_parameter('table_half').value
        self.table_step = self.get_parameter('table_step').value
        self.obstacle_x = self.get_parameter('obstacle_x').value
        self.obstacle_y = self.get_parameter('obstacle_y').value
        self.obstacle_z = self.get_parameter('obstacle_z').value
        self.obstacle_size = self.get_parameter('obstacle_size').value

        self.cloud_pub = self.create_publisher(
            PointCloud2, '/yolo/points_filtered', 10)
        self.create_timer(1.0 / self.rate, self.publish)
        self.get_logger().info(
            f'Virtual obstacle cloud ready: table z={self.table_z}, '
            f'obstacle at ({self.obstacle_x},{self.obstacle_y},'
            f'{self.obstacle_z}) size={self.obstacle_size}')

    def _table_points(self):
        n = int(2.0 * self.table_half / self.table_step)
        u = np.linspace(-self.table_half, self.table_half, n)
        uu, vv = np.meshgrid(u, u)
        pts = np.stack(
            [uu.ravel(), vv.ravel(),
             np.full(uu.size, self.table_z)], axis=1)
        return pts

    def _box_points(self):
        s = self.obstacle_size / 2.0
        cx, cy, cz = self.obstacle_x, self.obstacle_y, self.obstacle_z
        n = int(self.obstacle_size / self.table_step)
        u = np.linspace(-s, s, n)
        faces = []
        faces.append(np.stack([cx + u, cy + u, np.full(n, cz + s)], axis=1))
        faces.append(np.stack([cx + u, cy + u, np.full(n, cz - s)], axis=1))
        faces.append(np.stack([cx + u, np.full(n, cy + s), cz + u], axis=1))
        faces.append(np.stack([cx + u, np.full(n, cy - s), cz + u], axis=1))
        faces.append(np.stack([np.full(n, cx + s), cy + u, cz + u], axis=1))
        faces.append(np.stack([np.full(n, cx - s), cy + u, cz + u], axis=1))
        return np.concatenate(faces, axis=0)

    def publish(self):
        points = np.concatenate([self._table_points(), self._box_points()])
        header = Header()
        header.frame_id = self.base_frame
        header.stamp = self.get_clock().now().to_msg()
        cloud = PointCloud2()
        cloud.header = header
        cloud.height = 1
        cloud.width = len(points)
        cloud.fields = [
            PointField(name='x', offset=0,
                       datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4,
                       datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8,
                       datatype=PointField.FLOAT32, count=1),
        ]
        cloud.point_step = 12
        cloud.row_step = cloud.point_step * cloud.width
        cloud.is_bigendian = False
        cloud.is_dense = True
        cloud.data = points.astype(np.float32).tobytes()
        self.cloud_pub.publish(cloud)


def main():
    rclpy.init()
    node = VirtualObstacleCloud()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
