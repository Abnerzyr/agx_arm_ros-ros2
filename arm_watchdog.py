#!/usr/bin/env python3
"""arm_watchdog.py — 机械臂运行中掉电看门狗（方案 A）

目的：电控在【运行中】掉电/断连后，ctrl 节点既不自动重连也不重新 home。
本看门狗监控 /arm/feedback/joint_states：
  - 若反馈【曾经正常出现过、随后持续缺失】超过阈值 → 判定运行中掉电
    → 杀掉 agx-arm 栈主进程，由 systemd Restart=always 重启
    → 新栈启动重新执行 auto_home，电控恢复后自动回 home。

刻意设计（避免空转）：
  - 若反馈【从未出现过】（启动即 enable 失败 / 电控从未连上）→ 不动作，
    交给启动探测(常驻等待)或人工，绝不无限重启。
用法：由 start_arm_prod.sh 在栈启动后拉起（继承 ROS 环境）。
"""
import os
import signal
import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

NO_ARM_TIMEOUT = 15.0   # 反馈曾正常后，持续缺失多少秒判掉电 (s)
CHECK_PERIOD = 2.0
CONFIRM_COUNT = 2       # 连续确认次数，防抖动误杀


class ArmWatchdog(Node):
    def __init__(self):
        super().__init__('arm_watchdog')
        self._last = None          # 最近一次反馈墙钟时间；None=从未收到
        self._miss = 0
        self.create_subscription(
            JointState, '/arm/feedback/joint_states', self._cb, 10)
        self.create_timer(CHECK_PERIOD, self._check)
        self.get_logger().info(
            f'arm_watchdog started: arm 反馈曾正常后缺失 '
            f'{NO_ARM_TIMEOUT:.0f}s 将重启栈触发 auto-home')

    def _cb(self, msg):
        self._last = time.time()

    def _check(self):
        if self._last is None:
            # 从未收到过反馈：不判断（避免 enable 失败/未连接造成重启循环）
            return
        gap = time.time() - self._last
        if gap < NO_ARM_TIMEOUT:
            self._miss = 0
            return
        self._miss += 1
        if self._miss >= CONFIRM_COUNT:
            self.get_logger().error(
                f'Arm joint feedback lost for {gap:.0f}s (running power loss); '
                'restarting arm stack to trigger auto-home')
            # 杀父进程链(start_arm_prod.sh) → systemd Restart=always 拉起
            try:
                os.kill(os.getppid(), signal.SIGKILL)
            except Exception:
                pass
            time.sleep(1)
            os._exit(1)


def main():
    rclpy.init()
    node = ArmWatchdog()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
