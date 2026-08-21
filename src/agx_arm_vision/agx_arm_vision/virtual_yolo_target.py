#!/usr/bin/env python3

import math

import rclpy
from geometry_msgs.msg import PoseStamped, Quaternion
from rclpy.node import Node
from scipy.spatial.transform import Rotation as R
from std_msgs.msg import ColorRGBA, Header
from visualization_msgs.msg import Marker


class VirtualYoloTarget(Node):
    """Simulation stand-in for yolo_grasp.

    Publishes a fixed /grasp_pose (top-down grasp, matching the real
    camera-aligned approach) and a negligible-size /yolo/target_box (only
    used for the grasp pairing / collision point). No point cloud is
    published so the simulated octomap stays empty.
    """

    def __init__(self):
        super().__init__('virtual_yolo_target')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('x', 0.50)
        self.declare_parameter('y', 0.0)
        self.declare_parameter('z', 0.24)
        self.declare_parameter('box_center_z', 0.27)
        self.declare_parameter('box_size', 0.001)
        self.declare_parameter('rate', 5.0)
        self.declare_parameter('grasp_yaw_deg', 90.0)

        self.base_frame = self.get_parameter('base_frame').value
        self.x = self.get_parameter('x').value
        self.y = self.get_parameter('y').value
        self.z = self.get_parameter('z').value
        self.box_center_z = self.get_parameter('box_center_z').value
        self.box_size = self.get_parameter('box_size').value
        self.grasp_yaw = math.radians(
            self.get_parameter('grasp_yaw_deg').value)

        self.grasp_pub = self.create_publisher(
            PoseStamped, '/grasp_pose', 10)
        self.box_pub = self.create_publisher(
            Marker, '/yolo/target_box', 10)

        self.create_timer(1.0 / self.get_parameter('rate').value,
                          self.publish)
        self.get_logger().info(
            f'Virtual yolo target ready: grasp=({self.x:.3f}, '
            f'{self.y:.3f}, {self.z:.3f}) box_z='
            f'{self.box_center_z:.3f} size={self.box_size:.3f}')

    def publish(self):
        now = self.get_clock().now().to_msg()
        header = Header()
        header.frame_id = self.base_frame
        header.stamp = now

        pose = PoseStamped()
        pose.header = header
        pose.pose.position.x = float(self.x)
        pose.pose.position.y = float(self.y)
        pose.pose.position.z = float(self.z)
        # top-down grasp: tcp z-axis points down (world -z) so the
        # executor's approach is from above; yaw rotates the gripper
        quat = (R.from_euler('z', self.grasp_yaw)
                * R.from_quat([1.0, 0.0, 0.0, 0.0])).as_quat()
        pose.pose.orientation = Quaternion(
            x=float(quat[0]), y=float(quat[1]),
            z=float(quat[2]), w=float(quat[3]))
        self.grasp_pub.publish(pose)

        box = Marker()
        box.header = header
        box.ns = 'grasp_target_box'
        box.id = 0
        box.type = Marker.CUBE
        box.action = Marker.ADD
        box.pose.position.x = float(self.x)
        box.pose.position.y = float(self.y)
        box.pose.position.z = float(self.box_center_z)
        box.pose.orientation.w = 1.0
        box.scale.x = float(self.box_size)
        box.scale.y = float(self.box_size)
        box.scale.z = float(self.box_size)
        box.color = ColorRGBA(r=1.0, g=0.8, b=0.0, a=0.5)
        self.box_pub.publish(box)


def main():
    rclpy.init()
    node = VirtualYoloTarget()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
