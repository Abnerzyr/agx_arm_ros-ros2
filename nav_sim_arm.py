#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""nav_sim_arm.py — 机械臂端手动"导航模拟器"。

模拟车侧(competition_node + arm_task_bridge)向机械臂发指令，
并把机械臂返回的 /arm_task_report 以及 executor 进度显示出来。

用法（先启动真机管线 ./start_shelf.sh）:
    source install/setup.bash
    python3 nav_sim_arm.py

按键:
    1 / 2 / 3  -> 发 /arm/task_command(层号)  模拟"导航到位→下抓取指令"
    r          -> 发 /arm/release_command     模拟"导航到放货区→放下指令"
    s          -> 发 /arm/shelf/skip_align (测试: 无 aruco 时跳过对准)
    p          -> 发 /arm/shelf/preset_home (测试: 用 home 预设位)
    a          -> 自动跑一轮: 发层1抓取 -> 等 PICK 成功 -> 模拟导航5s
                  -> 发 release -> 等 PLACE 成功
    h          -> 帮助
    q          -> 退出
"""

import threading
import time

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Empty, Int32, String

# 机械臂 executor 状态名（与 grasp_executor.py / shelf_workflow_node.py 一致）
EXECUTOR_STATES = {
    0: 'IDLE', 1: 'OPEN_GRIPPER', 2: 'MOVE_TO_TARGET', 3: 'WAIT_REACH',
    4: 'CLOSE_GRIPPER', 5: 'MOVE_HOME', 6: 'WAIT_RELEASE',
    7: 'MOVE_TO_PLACE_ABOVE', 8: 'LOWER_TO_PLACE', 9: 'PLACE_OPEN',
    10: 'PLACE_LIFT', 11: 'LIFT_VERIFY',
}
# grasp_result 语义
GRASP_RESULTS = {0: 'FAIL_BEFORE_GRASP', 1: 'EMPTY_GRASP', 2: 'OBJECT_HELD'}
# 错误码
ERRORS = {
    0: 'SUCCESS', 2: 'INVALID_GOAL', 3: 'OBJECT_NOT_FOUND',
    5: 'GRASP_FAILED', 7: 'TIMEOUT', 8: 'CANCELED', 9: 'CONTROLLER_ERROR',
}

CYAN = '\033[36m'; GREEN = '\033[32m'; YELLOW = '\033[33m'
RED = '\033[31m'; BOLD = '\033[1m'; DIM = '\033[2m'; END = '\033[0m'


class NavSimArm(Node):

    def __init__(self):
        super().__init__('nav_sim_arm')
        # 发指令（与 arm_task_bridge 相同的绝对话题）
        self.task_cmd_pub = self.create_publisher(Int32, '/arm/task_command', 10)
        self.release_pub = self.create_publisher(
            Empty, '/arm/release_command', 10)
        self.skip_align_pub = self.create_publisher(
            Empty, '/arm/shelf/skip_align', 10)
        self.preset_home_pub = self.create_publisher(
            Empty, '/arm/shelf/preset_home', 10)

        # 收机械臂上报/进度
        self.report_sub = self.create_subscription(
            String, '/arm_task_report', self._on_report, 10)
        self.state_sub = self.create_subscription(
            Int32, '/arm/grasp_executor_state', self._on_exec_state, 10)
        self.result_sub = self.create_subscription(
            Int32, '/arm/grasp_result', self._on_grasp_result, 10)

        self._lock = threading.Lock()
        self._reports = {}          # command -> dict(success,error_code,message,at)
        self._exec_state = None
        self._last_exec_state = None
        self._last_result = None
        self._seq = 0

        self.get_logger().info('Nav simulator ready; h=帮助')

    # ---------- 发布 ----------
    def send_layer(self, layer):
        self._seq += 1
        msg = Int32()
        msg.data = int(layer)
        self.task_cmd_pub.publish(msg)
        print('%s[NAV] #%d send /arm/task_command = %d '
              '(模拟: 导航到位 → 抓取层%d)%s'
              % (CYAN, self._seq, layer, layer, END), flush=True)

    def send_release(self):
        self._seq += 1
        self.release_pub.publish(Empty())
        print('%s[NAV] #%d send /arm/release_command '
              '(模拟: 导航到放货区 → 放下)%s'
              % (CYAN, self._seq, END), flush=True)

    def send_skip_align(self):
        self.skip_align_pub.publish(Empty())
        print('%s[NAV] send /arm/shelf/skip_align (测试开关, 下次任务生效)%s'
              % (YELLOW, END), flush=True)

    def send_preset_home(self):
        self.preset_home_pub.publish(Empty())
        print('%s[NAV] send /arm/shelf/preset_home (测试开关, 下次任务生效)%s'
              % (YELLOW, END), flush=True)

    # ---------- 回调 ----------
    def _on_report(self, msg):
        import json
        try:
            d = json.loads(msg.data)
        except (TypeError, ValueError):
            print('%s[REPORT] (parse error): %r%s' % (RED, msg.data, END),
                  flush=True)
            return
        command = str(d.get('command', '?'))
        success = bool(d.get('success', False))
        code = int(d.get('error_code', -1))
        text = str(d.get('message', ''))
        err = ERRORS.get(code, str(code))
        with self._lock:
            self._reports[command] = dict(
                success=success, error_code=code, message=text,
                at=time.time())

        if command == 'STOW':
            tag = '可动(臂已收)'
            color = GREEN if success else RED
        elif success:
            tag = '成功'
            color = GREEN
        else:
            tag = '失败'
            color = RED
        mark = '✓' if success else '✗'
        print('%s[REPORT] %s %s %s command=%s success=%s '
              'error_code=%d(%s) message="%s"%s'
              % (BOLD, color, mark, tag, command, success, code, err, text,
                 END), flush=True)

    def _on_exec_state(self, msg):
        self._exec_state = int(msg.data)
        name = EXECUTOR_STATES.get(self._exec_state, str(self._exec_state))
        if self._exec_state != self._last_exec_state:
            self._last_exec_state = self._exec_state
            print('%s[ARM] executor state -> %d (%s)%s'
                  % (DIM, self._exec_state, name, END), flush=True)

    def _on_grasp_result(self, msg):
        val = int(msg.data)
        if val != self._last_result:
            self._last_result = val
            name = GRASP_RESULTS.get(val, str(val))
            color = GREEN if val == 2 else (RED if val == 1 else YELLOW)
            print('%s[ARM] grasp_result = %d (%s)%s'
                  % (color, val, name, END), flush=True)

    # ---------- 辅助 ----------
    def last_report(self, command):
        with self._lock:
            return self._reports.get(command)

    def wait_report(self, command, timeout):
        """等一条 command 上报(成功即返回 True；失败/超时返回 False)。"""
        deadline = time.time() + timeout
        while time.time() < deadline:
            rep = self.last_report(command)
            if rep and time.time() - rep['at'] < 2.0:
                return rep['success']
            time.sleep(0.1)
        print('%s[NAV] 等待 %s 上报超时 (%ds)%s'
              % (RED, command, timeout, END), flush=True)
        return False

    def auto_cycle(self, layer):
        """自动模拟一轮: 抓取(layer) → 等PICK成功 → 模拟导航5s → 放下 → 等PLACE。"""
        print('%s\n===== 自动流程开始 (层%d) =====%s' % (BOLD, layer, END),
              flush=True)
        self.send_layer(layer)
        if not self.wait_report('PICK', 120):
            print('%s自动流程: PICK 未成功, 中止%s' % (RED, END), flush=True)
            return
        print('%s[NAV] PICK 成功 → 模拟导航去放货区 5s ...%s' % (YELLOW, END),
              flush=True)
        time.sleep(5.0)
        self.send_release()
        if not self.wait_report('PLACE', 120):
            print('%s自动流程: PLACE 未成功, 中止%s' % (RED, END), flush=True)
            return
        print('%s===== 自动流程完成 =====%s' % (GREEN, END), flush=True)


def print_help():
    print(BOLD + '\n按键帮助:' + END)
    print('  1/2/3 : 发 task_command(层号) → 机械臂对准+识别+抓取+回home')
    print('  r     : 发 release_command → 机械臂放置+回home')
    print('  s     : 发 skip_align    p: 发 preset_home  (测试开关)')
    print('  a     : 自动跑一轮完整抓-放(层1)')
    print('  h     : 帮助     q: 退出')
    print('观察: [REPORT] 即机械臂上报(PICK/PLACE/STOW)，'
          '[ARM] 为 executor 进度。\n')


def main(args=None):
    rclpy.init(args=args)
    node = NavSimArm()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    spin = threading.Thread(target=executor.spin, daemon=True)
    spin.start()

    print_help()
    auto = None
    try:
        while True:
            try:
                key = input('nav> ').strip().lower()
            except EOFError:
                break
            if not key:
                continue
            if key in ('1', '2', '3'):
                node.send_layer(int(key))
            elif key == 'r':
                node.send_release()
            elif key == 's':
                node.send_skip_align()
            elif key == 'p':
                node.send_preset_home()
            elif key == 'a':
                auto = threading.Thread(
                    target=node.auto_cycle, args=(1,), daemon=True)
                auto.start()
            elif key == 'h':
                print_help()
            elif key == 'q':
                break
            else:
                print('未知按键 %r (h=帮助)' % key)
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
