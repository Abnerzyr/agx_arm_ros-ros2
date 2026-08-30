#!/usr/bin/env python3

"""完整 grasp+place 训练循环驱动（人工上报每轮结果）。

每轮：触发抓取 → executor 自动完成 grasp+place（test_flow）→ 回 IDLE →
等待人工上报 manual_round_result（1空夹/2未夹稳/3未放平/4放平）→
收到后自动触发下一轮。等待上报期间发布 /arm/manual_awaiting=True。

若 executor 全程未离开 IDLE（校验/规划失败、机械臂没动）→ 自动跳过，
不进入"等上报"，等 post_round_wait 后直接下一轮。开机等 startup_settle，
每轮（含跳过）结束等 post_round_wait 再进入下一轮。
"""

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from std_msgs.msg import Bool, Empty, Int32
from visualization_msgs.msg import Marker


class GraspTrainDriver(Node):
    PHASE_START = 0
    PHASE_TRIGGER = 1
    PHASE_AWAIT_DONE = 2
    PHASE_WAIT_MANUAL = 3

    def __init__(self):
        super().__init__('grasp_train_driver')
        self.declare_parameter('startup_settle', 15.0)
        self.declare_parameter('post_round_wait', 15.0)
        self.declare_parameter('max_rounds', 0)
        self.declare_parameter('detect_fresh', 1.0)
        self.declare_parameter('min_round_time', 15.0)
        self.declare_parameter('round_timeout', 180.0)
        self.declare_parameter('manual_timeout', 300.0)

        self.startup_settle = float(
            self.get_parameter('startup_settle').value)
        self.post_round_wait = float(
            self.get_parameter('post_round_wait').value)
        self.max_rounds = int(self.get_parameter('max_rounds').value)
        self.detect_fresh = float(self.get_parameter('detect_fresh').value)
        self.min_round_time = float(self.get_parameter('min_round_time').value)
        self.round_timeout = float(self.get_parameter('round_timeout').value)
        self.manual_timeout = float(self.get_parameter('manual_timeout').value)

        self.trigger_pub = self.create_publisher(
            Empty, 'manual_grasp_start', 10)
        self.awaiting_pub = self.create_publisher(
            Bool, 'manual_awaiting', 10)

        self.create_subscription(
            Marker, 'yolo/target_box', self._box_cb, 10)
        self.create_subscription(
            PoseStamped, 'grasp_pose', self._pose_cb, 10)
        self.create_subscription(
            Int32, 'grasp_executor_state', self._state_cb, 10)
        self.create_subscription(
            Int32, 'manual_round_result', self._manual_cb, 10)

        self._last_box = 0.0
        self._last_pose = 0.0
        self._exec_state = 0
        self._manual_time = 0.0
        self._manual_code = 0
        self._phase = self.PHASE_START
        self._start_at = 0.0
        self._next_trigger_at = 0.0
        self._round = 0
        self._round_start = 0.0
        self._was_running = False
        self._stopped = False

        self.create_timer(0.5, self._tick)
        self.get_logger().info('Grasp train driver ready (manual report loop)')

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _box_cb(self, msg):
        del msg
        self._last_box = self._now()

    def _pose_cb(self, msg):
        del msg
        self._last_pose = self._now()

    def _state_cb(self, msg):
        self._exec_state = int(msg.data)

    def _manual_cb(self, msg):
        self._manual_time = self._now()
        self._manual_code = int(msg.data)

    def _tick(self):
        await_b = Bool()
        await_b.data = (self._phase == self.PHASE_WAIT_MANUAL)
        self.awaiting_pub.publish(await_b)
        if self._stopped:
            return
        now = self._now()

        if self._phase == self.PHASE_START:
            if self._start_at == 0.0:
                self._start_at = now + self.startup_settle
                self.get_logger().info(
                    f'[TRAIN] startup settle {self.startup_settle:.0f}s')
            elif now >= self._start_at:
                self._phase = self.PHASE_TRIGGER
                self._next_trigger_at = now
            return

        if self.max_rounds > 0 and self._round >= self.max_rounds:
            self._finish('max_rounds reached')
            return

        if self._phase == self.PHASE_TRIGGER:
            if now < self._next_trigger_at:
                return
            if self._exec_state != 0:
                return
            if (now - self._last_box > self.detect_fresh
                    or now - self._last_pose > self.detect_fresh):
                return
            self._round += 1
            self._round_start = now
            self._was_running = False
            self.trigger_pub.publish(Empty())
            self._phase = self.PHASE_AWAIT_DONE
            self.get_logger().info(
                f'[TRAIN] round {self._round} triggered')
            return

        if self._phase == self.PHASE_AWAIT_DONE:
            if self._exec_state != 0:
                self._was_running = True
            if now - self._round_start > self.min_round_time:
                if self._was_running and self._exec_state == 0:
                    self._phase = self.PHASE_WAIT_MANUAL
                    self.get_logger().warn(
                        f'[TRAIN] round {self._round} done; WAITING for '
                        'manual report (1空夹 2未夹稳 3未放平 4放平)')
                    return
                if not self._was_running:
                    # 全程未离开 IDLE（校验/规划失败、机械臂没动）→ 自动跳过
                    self.get_logger().warning(
                        f'[TRAIN] round {self._round} no motion; skip '
                        f'(wait {self.post_round_wait:.0f}s)')
                    self._phase = self.PHASE_TRIGGER
                    self._next_trigger_at = now + self.post_round_wait
                    return
            if now - self._round_start > self.round_timeout:
                self.get_logger().warning(
                    f'[TRAIN] round {self._round} not done within '
                    f'{self.round_timeout:.0f}s; stopping')
                self._finish('round timeout')
            return

        if self._phase == self.PHASE_WAIT_MANUAL:
            if self._manual_time > self._round_start:
                self.get_logger().info(
                    f'[TRAIN] round {self._round} manual code='
                    f'{self._manual_code}; next round '
                    f'(wait {self.post_round_wait:.0f}s)')
                self._phase = self.PHASE_TRIGGER
                self._next_trigger_at = now + self.post_round_wait
                return
            if now - self._round_start > self.manual_timeout:
                self.get_logger().error(
                    '[TRAIN] manual report timeout; stopping')
                self._finish('manual report timeout')
            return

    def _finish(self, reason):
        self._stopped = True
        self.get_logger().info(f'[TRAIN] STOP ({reason})')


def main():
    rclpy.init()
    node = GraspTrainDriver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
