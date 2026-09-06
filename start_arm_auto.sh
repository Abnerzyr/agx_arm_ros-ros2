#!/bin/bash
# start_arm_auto.sh : systemd 包装脚本（仿车侧 chassis/task-scheduler 模式）
#
# 由 agx-arm.service 在开机时以 User=s1 执行。
# 可选读取同目录 arm_mode.conf（仅 shell 变量赋值）切换要启动的臂脚本：
#   ARM_START=start_arm_prod.sh     # 默认：真机生产版（无 rviz/marker）
#   ARM_START=start_shelf.sh        # 调试版（含 rviz，手动调试时用）
set -e

DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
MODE_FILE="$DIR/arm_mode.conf"

ARM_START=start_arm_prod.sh
if [ -f "$MODE_FILE" ]; then
    source "$MODE_FILE"
fi

exec /bin/bash "$DIR/$ARM_START"
