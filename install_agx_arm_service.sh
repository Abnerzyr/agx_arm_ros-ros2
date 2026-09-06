#!/bin/bash
# install_agx_arm_service.sh : 一键安装 agx-arm 开机自启（一次性，需 sudo）
#
# 做的事:
#   1) 安装 /etc/systemd/system/agx-arm.service
#   2) 安装 /etc/sudoers.d/arm-can —— 只放行臂启动脚本用到的 CAN 免密命令
#      (systemd 以 s1 运行无终端，sudo 不能弹密码，必须 NOPASSWD 白名单)
#   3) systemctl daemon-reload
#
# 不会自动 enable（是否开机自启由你决定）：
#   启用开机自启:      sudo systemctl enable --now agx-arm
#   只手动启/停:       sudo systemctl start agx-arm   /   sudo systemctl stop agx-arm
#   查看状态/日志:     systemctl status agx-arm        journalctl -u agx-arm -f
set -e

DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

echo "== 1/3 安装 unit =="
sudo install -m 644 "$DIR/agx-arm.service" /etc/systemd/system/agx-arm.service

echo "== 2/3 安装 sudoers 白名单 (NOPASSWD CAN) =="
sudo tee /etc/sudoers.d/arm-can >/dev/null <<'EOF'
# 仅供 agx-arm.service(s1) 启动臂时配置 CAN 接口免密
# 同时放行 /sbin/ip 与 /usr/sbin/ip（不同发行版路径可能不同）
s1 ALL=(root) NOPASSWD: /sbin/ip link set can0 down, \
                         /sbin/ip link set can0 up type can bitrate 1000000, \
                         /sbin/ip link set can1 down, \
                         /sbin/ip link set can1 up type can bitrate 1000000, \
                         /usr/sbin/ip link set can0 down, \
                         /usr/sbin/ip link set can0 up type can bitrate 1000000, \
                         /usr/sbin/ip link set can1 down, \
                         /usr/sbin/ip link set can1 up type can bitrate 1000000
EOF
sudo chmod 440 /etc/sudoers.d/arm-can

echo "== 3/3 reload =="
sudo systemctl daemon-reload

echo ""
echo "OK. 后续操作（任选）："
echo "  开机自启:      sudo systemctl enable --now agx-arm"
echo "  仅手动启动:    sudo systemctl start agx-arm"
echo "  查看日志:      journalctl -u agx-arm -f"
echo "  说明: 比赛日请先让 agx-arm 就绪(周期 STOW) 再启动车侧调度器下单"
