#!/usr/bin/env python3

"""Grasp-only training loop driver.

Drives grasp_executor directly for grasp-only RL training: waits until a
fresh yolo detection (target box + grasp pose) is available while the
executor is IDLE, then publishes `manual_grasp_start`. Waits for the
`grasp_result` (reward attribution is handled inside yolo_grasp's RL
refiner) and for the executor to return to IDLE (grasp_only mode releases
the object back onto the shelf), then starts the next attempt.

This node intentionally does NOT use shelf_workflow / aruco alignment /
place_planner: the training loop is grasp-only and relies on the object
being within the camera field of view at the home pose.
"""

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from std_msgs.msg import Empty, Int32
from visualization_msgs.msg import Marker


class GraspTrainDriver(Node):
    PHASE_START = 0
    PHASE_DETECT = 1
    PHASE_AWAIT_RESULT = 2
    PHASE_AWAIT_IDLE = 3

    def __init__(self):
        super().__init__('grasp_train_driver')
        self.declare_parameter('warmup', 5.0)
        self.declare_parameter('settle', 3.0)
        self.declare_parameter('max_attempts', 100)
        self.declare_parameter('stop_on_failures', 5)
        self.declare_parameter('result_timeout', 180.0)
        self.declare_parameter('detect_fresh', 1.0)

        self.warmup = float(self.get_parameter('warmup').value)
        self.settle = float(self.get_parameter('settle').value)
        self.max_attempts = int(self.get_parameter('max_attempts').value)
        self.stop_on_failures = int(
            self.get_parameter('stop_on_failures').value)
        self.result_timeout = float(
            self.get_parameter('result_timeout').value)
        self.detect_fresh = float(self.get_parameter('detect_fresh').value)

        self.trigger_pub = self.create_publisher(
            Empty, 'manual_grasp_start', 10)

        self.create_subscription(
            Marker, 'yolo/target_box', self._box_cb, 10)
        self.create_subscription(
            PoseStamped, 'grasp_pose', self._pose_cb, 10)
        self.create_subscription(
            Int32, 'grasp_result', self._result_cb, 10)
        self.create_subscription(
            Int32, 'grasp_executor_state', self._state_cb, 10)

        self._last_box = 0.0
        self._last_pose = 0.0
        self._exec_state = 0
        self._phase = self.PHASE_START
        self._warmup_until = 0.0
        self._attempt = 0
        self._trigger_time = 0.0
        self._idle_since = None
        self._consec_fail = 0
        self._stopped = False
        self._stats = {'succ': 0, 'empty': 0, 'fail': 0, 'timeout': 0}

        self.create_timer(0.5, self._tick)
        self.get_logger().info(
            f'Grasp train driver ready: max_attempts={self.max_attempts} '
            f'stop_on_failures={self.stop_on_failures} '
            f'settle={self.settle}s')

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _box_cb(self, msg):
        del msg
        self._last_box = self._now()

    def _pose_cb(self, msg):
        del msg
        self._last_pose = self._now()

    def _result_cb(self, msg):
        if self._phase != self.PHASE_AWAIT_RESULT:
            return
        code = int(msg.data)
        if code == 2:
            self._stats['succ'] += 1
            self._consec_fail = 0
        elif code == 1:
            self._stats['empty'] += 1
            self._consec_fail += 1
        else:
            self._stats['fail'] += 1
            self._consec_fail += 1
        self.get_logger().info(
            f'[TRAIN] attempt {self._attempt} result={code} '
            f'(succ={self._stats["succ"]} empty={self._stats["empty"]} '
            f'fail={self._stats["fail"]})')
        self._phase = self.PHASE_AWAIT_IDLE
        self._idle_since = None

    def _state_cb(self, msg):
        self._exec_state = int(msg.data)

    def _tick(self):
        if self._stopped:
            return
        now = self._now()

        if self._phase == self.PHASE_START:
            if self._warmup_until == 0.0:
                self._warmup_until = now + self.warmup
                self.get_logger().info(
                    f'[TRAIN] warmup {self.warmup:.0f}s, then auto-triggering')
            elif now >= self._warmup_until:
                self._phase = self.PHASE_DETECT
            return

        if self._attempt > self.max_attempts:
            self._finish('max_attempts reached')
            return
        if self._consec_fail >= self.stop_on_failures:
            self._finish(f'{self._consec_fail} consecutive failures')
            return

        if self._phase == self.PHASE_DETECT:
            if self._exec_state != 0:
                return
            if (now - self._last_box > self.detect_fresh
                    or now - self._last_pose > self.detect_fresh):
                return
            self._attempt += 1
            self.trigger_pub.publish(Empty())
            self._trigger_time = now
            self._phase = self.PHASE_AWAIT_RESULT
            self.get_logger().info(
                f'[TRAIN] triggered grasp {self._attempt}/{self.max_attempts}')
            return

        if self._phase == self.PHASE_AWAIT_RESULT:
            if now - self._trigger_time > self.result_timeout:
                self._stats['timeout'] += 1
                self._consec_fail += 1
                self.get_logger().warning(
                    f'[TRAIN] attempt {self._attempt} timed out after '
                    f'{self.result_timeout:.0f}s')
                self._phase = self.PHASE_AWAIT_IDLE
                self._idle_since = None
            return

        if self._phase == self.PHASE_AWAIT_IDLE:
            if self._exec_state == 0:
                if self._idle_since is None:
                    self._idle_since = now
                elif now - self._idle_since >= self.settle:
                    self._idle_since = None
                    self._phase = self.PHASE_DETECT
            else:
                self._idle_since = None
            return

    def _finish(self, reason):
        self._stopped = True
        s = self._stats
        total = s['succ'] + s['empty'] + s['fail'] + s['timeout']
        self.get_logger().info(
            f'[TRAIN] STOP ({reason}): total={total} '
            f'succ={s["succ"]} empty={s["empty"]} fail={s["fail"]} '
            f'timeout={s["timeout"]} rate={s["succ"] / max(total, 1):.2f}')


def main():
    rclpy.init()
    node = GraspTrainDriver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
