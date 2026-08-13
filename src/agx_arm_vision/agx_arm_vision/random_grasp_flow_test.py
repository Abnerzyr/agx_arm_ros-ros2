#!/usr/bin/env python3

import math
import random

import rclpy
from geometry_msgs.msg import PoseStamped, Quaternion
from rclpy.node import Node
from scipy.spatial.transform import Rotation as R
from std_msgs.msg import Empty
from tf2_ros import Buffer, TransformException, TransformListener

from agx_arm_msgs.msg import GripperStatus
from agx_arm_vision.moveit2_local import MoveIt2


class RandomGraspFlowNode(Node):
    PUBLISH_WAIT = 1.0
    FORCE_THRESHOLD = 1.5
    HOLD_WIDTH_MIN = 0.005

    def __init__(self):
        super().__init__('random_grasp_flow')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('end_effector', 'tcp_link')
        self.declare_parameter('max_step', 0.35)
        self.declare_parameter('min_z', 0.30)
        self.declare_parameter('max_z', 0.60)
        self.declare_parameter('max_radius', 0.50)
        self.declare_parameter('grasp_wait', 25.0)
        self.declare_parameter('open_wait', 3.0)

        self.base_frame = self.get_parameter('base_frame').value
        self.end_effector = self.get_parameter('end_effector').value
        self.max_step = self.get_parameter('max_step').value
        self.min_z = self.get_parameter('min_z').value
        self.max_z = self.get_parameter('max_z').value
        self.max_radius = self.get_parameter('max_radius').value
        self.grasp_wait = self.get_parameter('grasp_wait').value
        self.open_wait = self.get_parameter('open_wait').value

        self.arm = MoveIt2(
            node=self,
            base_link=self.base_frame,
            end_effector=self.end_effector,
            group_name='arm',
            constrain_orientation=True,
        )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.pose_pub = self.create_publisher(
            PoseStamped, '/grasp_pose', 10)
        self.start_pub = self.create_publisher(
            Empty, '/manual_grasp_start', 10)
        self.release_pub = self.create_publisher(
            Empty, '/manual_release', 10)
        self.create_subscription(
            GripperStatus, '/feedback/gripper_status',
            self.status_callback, 10)

        self.last_pos = None
        self.pending_pose = None
        self.cycle = 0
        self.phase = 'idle'
        self.deadline = 0.0
        self.width_min = None
        self.force_max = 0.0
        self.current_width = None
        self.release_retries = 0
        self.cur_target = None
        self.cur_theta = 0.0

        self.create_timer(0.2, self.tick)
        self.get_logger().info(
            f'Random grasp flow ready (max_step={self.max_step:.2f} m, '
            f'z=[{self.min_z:.2f},{self.max_z:.2f}], '
            f'r<={self.max_radius:.2f}, grasp_wait={self.grasp_wait:.1f} s)')

    def status_callback(self, msg):
        self.current_width = msg.width
        if self.phase != 'started':
            return
        if self.width_min is None:
            self.width_min = msg.width
        else:
            self.width_min = min(self.width_min, msg.width)
        self.force_max = max(self.force_max, msg.force)

    def tick(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        if self.phase == 'idle':
            self._start_check()
        elif self.phase == 'checking':
            if not self.arm.is_done():
                return
            if self.arm.success:
                self.pose_pub.publish(self.pending_pose)
                self.cycle += 1
                p = self.pending_pose.pose.position
                self.cur_target = (p.x, p.y, p.z)
                self.get_logger().info(
                    f'#{self.cycle} target=({p.x:.3f},{p.y:.3f},{p.z:.3f}) '
                    f'theta={self.cur_theta:.0f} deg')
                self.phase = 'published'
                self.deadline = now + self.PUBLISH_WAIT
            else:
                p = self.pending_pose.pose.position
                self.get_logger().info(
                    f'skip unreachable ({p.x:.3f},{p.y:.3f},{p.z:.3f})')
                self.phase = 'idle'
        elif self.phase == 'published' and now >= self.deadline:
            self.start_pub.publish(Empty())
            self.phase = 'started'
            self.deadline = now + self.grasp_wait
            self.width_min = None
            self.force_max = 0.0
        elif self.phase == 'started' and now >= self.deadline:
            self.release_pub.publish(Empty())
            self._log_outcome()
            self.phase = 'releasing'
            self.release_retries = 0
            self.deadline = now + 1.0
        elif self.phase == 'releasing':
            if (self.current_width is not None
                    and self.current_width >= 0.09):
                self.phase = 'released'
                self.deadline = now + self.open_wait
            elif now >= self.deadline:
                self.release_retries += 1
                if self.release_retries >= 15:
                    self.get_logger().warn(
                        'release timeout; advancing to next cycle')
                    self.phase = 'released'
                    self.deadline = now + self.open_wait
                else:
                    self.release_pub.publish(Empty())
                    self.deadline = now + 1.0
        elif self.phase == 'released' and now >= self.deadline:
            self.phase = 'idle'

    def _start_check(self):
        try:
            t = self.tf_buffer.lookup_transform(
                self.base_frame, self.end_effector,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=1.0),
            )
        except TransformException:
            self.get_logger().warn(
                'TF unavailable; will retry on next tick')
            return

        if self.last_pos is None:
            self.last_pos = (
                t.transform.translation.x,
                t.transform.translation.y,
                t.transform.translation.z,
            )
        px = self.last_pos[0] + random.uniform(-self.max_step, self.max_step)
        py = self.last_pos[1] + random.uniform(-self.max_step, self.max_step)
        pz = self.last_pos[2] + random.uniform(-self.max_step, self.max_step)
        pz = max(self.min_z, min(self.max_z, pz))
        r = math.hypot(px, py)
        if r > self.max_radius:
            scale = self.max_radius / r
            px *= scale
            py *= scale
        self.last_pos = (px, py, pz)

        theta = random.uniform(-math.pi, math.pi)
        self.cur_theta = math.degrees(theta)
        q_tcp = t.transform.rotation
        rot = R.from_quat(
            [q_tcp.x, q_tcp.y, q_tcp.z, q_tcp.w]) * R.from_euler('z', theta)
        q = rot.as_quat()

        pose = PoseStamped()
        pose.header.frame_id = self.base_frame
        pose.pose.position.x = px
        pose.pose.position.y = py
        pose.pose.position.z = pz
        pose.pose.orientation = Quaternion(
            x=q[0], y=q[1], z=q[2], w=q[3])

        self.pending_pose = pose
        self.arm.move_to_pose(pose.pose, self.base_frame, plan_only=True)
        self.phase = 'checking'

    def _log_outcome(self):
        if self.width_min is None:
            verdict = 'no_feedback'
        elif (self.force_max > self.FORCE_THRESHOLD
                and self.width_min > self.HOLD_WIDTH_MIN):
            verdict = 'held'
        elif self.width_min < self.HOLD_WIDTH_MIN:
            verdict = 'closed_empty'
        else:
            verdict = 'no_grasp'
        self.get_logger().info(
            f'#{self.cycle} result: width_min='
            f'{self.width_min if self.width_min is not None else -1:.3f} '
            f'force_max={self.force_max:.1f} -> {verdict}')


def main():
    rclpy.init()
    node = RandomGraspFlowNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
