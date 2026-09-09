#!/bin/bash
# start_arm_prod.sh : 真机生产启动脚本（裁剪版 start_shelf.sh）
#
# 与 start_shelf.sh 的差异:
#   - 删除 RViz（manual_rviz.launch.py）—— 生产无显示
#   - 删除 grasp_target_marker —— 纯 RViz 显示转发
#   - aruco_tracker 保留为【注释】：
#       仅在 skip_align=false(需要 aruco 精对准) 时才需要；当前默认 skip_align=true 不依赖 aruco。
#       如需启用：取消下面 "=== Starting ArUco tracker ===" 段的注释。
#   - 其余节点/参数/日志与 start_shelf.sh 一致（MoveIt+/arm、realsense、yolo、place_planner、
#     grasp_executor、shelf_workflow）
#
# /arm_task_report 由 shelf_workflow 内部发布，无需额外节点。
cd /home/s1/tiaozhanbei/agx_arm_ros-ros2
source install/setup.bash

echo "=== Activating CAN ==="
for iface in can0 can1; do
    sudo ip link set $iface down 2>/dev/null || true
    sudo ip link set $iface up type can bitrate 1000000 2>/dev/null || true
done
sleep 0.5

echo "=== Waiting for CAN interfaces (USB-CAN may enumerate late) ==="
for i in $(seq 1 20); do
    [ -e /sys/class/net/can0 ] && break
    [ -e /sys/class/net/can1 ] && break
    sleep 1
done

echo "=== Auto-detecting arm CAN port (常驻等待，上电自动接管) ==="
CAN_PORT=""
cycle=0
while [ -z "$CAN_PORT" ]; do
    cycle=$((cycle + 1))
    for iface in can0 can1; do
        sudo ip link set $iface down 2>/dev/null || true
        sudo ip link set $iface up type can bitrate 1000000 2>/dev/null || true
    done
    sleep 0.5
    for iface in can0 can1; do
        HAS_ERR=$(ip -details link show $iface 2>/dev/null | grep -c 'BUS-OFF\|NO-CARRIER' || true)
        if [ "$HAS_ERR" -gt 0 ]; then
            echo "  $iface: BUS-OFF/NO-CARRIER, skip"
            continue
        fi
        echo "  $iface: probing..."
        RESULT=$(timeout 8 python3 -c "
import sys
from pyAgxArm import create_agx_arm_config, AgxArmFactory, ArmModel
for ver in ('default', 'v112'):
    try:
        cfg = create_agx_arm_config(robot=ArmModel.NERO, comm='can',
                                    channel='$iface', firmeware_version=ver)
        arm = AgxArmFactory.create_arm(cfg)
        arm.connect()
        s = arm.get_joints_enable_status_list()
        arm.disconnect()
        if isinstance(s, list) and len(s) >= 7:
            print('OK')
            sys.exit(0)
    except Exception:
        continue
print('FAIL')
" 2>/dev/null || echo 'FAIL')
        if [ "$RESULT" = "OK" ]; then
            CAN_PORT=$iface
            echo "  $iface: arm found!"
            break
        else
            echo "  $iface: no arm (result=$RESULT)"
        fi
    done
    [ -n "$CAN_PORT" ] && break
    WAIT_MIN=$((cycle * 20 / 60))
    if (( cycle % 15 == 0 )); then
        echo "[wait] arm not found after ~${WAIT_MIN} min; waiting for power/USB-CAN..."
    else
        echo "  no arm yet (cycle $cycle, ~${WAIT_MIN} min); waiting 20s..."
    fi
    sleep 20
done

echo "=== Resetting CAN port state ==="
sudo ip link set $CAN_PORT down 2>/dev/null || true
sleep 1
sudo ip link set $CAN_PORT up type can bitrate 1000000 2>/dev/null || true
sleep 2

echo "=== Killing old processes ==="
pkill -f start_single_agx_arm_moveit 2>/dev/null || true
pkill -9 -f move_group 2>/dev/null || true
pkill -9 -f ros2_control_node 2>/dev/null || true
pkill -9 -f rviz2 2>/dev/null || true
pkill -9 -f agx_arm_ctrl_single 2>/dev/null || true
pkill -9 -f realsense2_camera 2>/dev/null || true
pkill -9 -f aruco_tracker 2>/dev/null || true
pkill -9 -f yolo_grasp 2>/dev/null || true
pkill -9 -f place_planner 2>/dev/null || true
pkill -9 -f grasp_target_marker 2>/dev/null || true
pkill -9 -f 'robot_state_publisher.*__ns:=/arm' 2>/dev/null || true
sleep 1
pkill -9 -f grasp_executor 2>/dev/null || true
pkill -9 -f vision_grasp_node 2>/dev/null || true
sleep 1

echo "=== Launching MoveIt + Arm ($CAN_PORT) in /arm namespace ==="
ros2 launch agx_arm_ctrl start_single_agx_arm_moveit.launch.py \
  can_port:=$CAN_PORT \
  arm_type:=nero \
  effector_type:=agx_gripper \
  auto_enable:=true \
  auto_control_gate:=true \
  speed_percent:=20 \
  fw_version:=v111 \
  auto_home:=true \
  pub_rate:=50 \
  use_rviz:=false \
  follow:=true \
  namespace:=arm &
MOVEIT_PID=$!

echo "=== Waiting for MoveIt ... ==="
sleep 10
echo "=== Verifying controllers ==="
for i in $(seq 1 10); do
    if ros2 action list 2>/dev/null | grep -q gripper; then
        echo "  controllers ready"
        break
    fi
    echo "  waiting for controllers... ($i)"
    sleep 2
done

echo "=== Starting camera (global /camera) ==="
ros2 run realsense2_camera realsense2_camera_node \
  --ros-args -r __node:=camera -r __ns:=/camera \
  -p align_depth.enable:=true \
  -p publish_tf:=false \
  -p depth_module.profile:=640x480x15 \
  -p rgb_camera.profile:=640x480x15 \
  -p enable_infra1:=false \
  -p enable_infra2:=false &
CAM_PID=$!

sleep 3

# === ArUco tracker (注释保留) ===
# 仅 skip_align=false(需要 aruco 精对准) 时启用；当前默认 skip_align=true 不使用。
# 取消下面注释即可开启：
# echo "=== Starting ArUco tracker ==="
# ros2 run aruco_opencv aruco_tracker_autostart \
#   --ros-args \
#   -p cam_base_topic:=/camera/camera/color/image_raw \
#   -p marker_dict:=4X4_50 \
#   -p marker_size:=0.05 &>/tmp/aruco.log &
# ARUCO_PID=$!
# echo "  aruco_tracker PID=$ARUCO_PID"

echo "=== Starting YOLO+Grasp (in /arm) ==="
OMP_NUM_THREADS=2 ros2 run agx_arm_vision yolo_grasp \
  --ros-args -r __ns:=/arm \
  -p base_frame:=arm/base_link \
  -p camera_optical_frame:=arm/camera_color_optical_frame \
  -p publish_viz:=false &>/tmp/yolo.log &
YOLO_PID=$!
echo "  yolo_grasp PID=$YOLO_PID"

echo "=== Starting place planner (in /arm) ==="
OPENBLAS_NUM_THREADS=2 ros2 run agx_arm_vision place_planner \
  --ros-args -r __ns:=/arm \
  -p base_frame:=arm/base_link \
  -p end_effector_link:=arm/tcp_link \
  -p camera_optical_frame:=arm/camera_color_optical_frame \
  -p process_period:=1.0 \
  -p publish_viz:=false &>/tmp/place.log &
PLACE_PID=$!
echo "  place_planner PID=$PLACE_PID"

sleep 2

echo "=== Starting grasp executor (in /arm) ==="
ros2 run agx_arm_vision grasp_executor \
  --ros-args -r __ns:=/arm \
  -p base_link:=arm/base_link \
  -p end_effector_link:=arm/tcp_link \
  -p publish_viz:=false &>/tmp/grasp.log &
GRASP_PID=$!
echo "  grasp_executor PID=$GRASP_PID"

echo "=== Starting shelf workflow (in /arm) ==="
ros2 run agx_arm_vision shelf_workflow \
  --ros-args -r __ns:=/arm \
  -p base_frame:=arm/base_link \
  -p end_effector_link:=arm/tcp_link \
  -p camera_frame:=arm/camera_color_optical_frame \
  -p publish_viz:=false \
  -p depth_mon_enable:=false &>/tmp/shelf.log &
SHELF_PID=$!
echo "  shelf_workflow PID=$SHELF_PID"

echo "=== All started (PROD, no rviz / no marker) ==="
echo "CAN=$CAN_PORT  MoveIt=$MOVEIT_PID  Cam=$CAM_PID  YOLO=$YOLO_PID  Place=$PLACE_PID  Grasp=$GRASP_PID  Shelf=$SHELF_PID"
echo ""
echo "Manual commands (注意 /arm/ 前缀):"
echo "  ros2 topic pub --once -w 1 /arm/task_command std_msgs/msg/Int32 '{data: 1}'"
echo "  ros2 topic pub --once -w 1 /arm/release_command std_msgs/msg/Empty '{}'"
echo "  ros2 topic pub --once -w 1 /arm/manual_release_force std_msgs/msg/Empty '{}'  # 原地松爪（手动兜底）"

echo "=== Starting arm watchdog (运行中掉电自动重启触发 auto-home) ==="
nohup python3 /home/s1/tiaozhanbei/agx_arm_ros-ros2/arm_watchdog.py >/tmp/arm_watchdog.log 2>&1 &
echo "  arm_watchdog PID=$!"

wait
