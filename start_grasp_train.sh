#!/bin/bash
# start_grasp_train.sh : 仅抓取 RL 训练（无对准、无放置，臂进 /arm 命名空间版）
#
# 与完整货架流程 start_shelf.sh 的区别：
#   - 不启动 aruco_tracker / shelf_workflow / place_planner（训练链路不含对准与放置）
#   - yolo_grasp 加 rl_enable:=true（进程内 RL 精修）
#   - grasp_executor 加 grasp_only:=true（抓完原地松开放回货架，直接回 home）
#   - AUTO=1 时启动 grasp_train_driver（自动循环触发抓取，无人值守）
#   - 物体需摆放在 home 位相机视野内
#
# 环境变量:
#   AUTO           =1 启动 grasp_train_driver（默认 0=手动触发，人监督）
#   TRAIN          =1 启动后自动开启 RL 训练（默认 0=评估模式，贪心精修不学习）
#   MAX_ATTEMPTS  auto 训练最大尝试次数（默认 30）
#   RL_DATA_DIR   RL 数据目录（默认 $PWD/grasp_rl_data，即工作空间内）
#
# 手动触发（AUTO=0 时）:
#   ros2 topic pub --once -w 1 /arm/manual_grasp_start std_msgs/msg/Empty '{}'
cd /home/s1/tiaozhanbei/agx_arm_ros-ros2
rm -f /dev/shm/fastrtps* 2>/dev/null
source install/setup.bash

AUTO=${AUTO:-0}
TRAIN=${TRAIN:-0}
MAX_ATTEMPTS=${MAX_ATTEMPTS:-30}
RL_DATA_DIR=${RL_DATA_DIR:-$PWD/grasp_rl_data}
export GRASP_RL_DATA_DIR=$RL_DATA_DIR

echo "=== RL data dir: $RL_DATA_DIR ==="
mkdir -p "$RL_DATA_DIR/checkpoint" "$RL_DATA_DIR/samples"

echo "=== Activating CAN ==="
for iface in can0 can1; do
    sudo ip link set $iface down 2>/dev/null || true
    sudo ip link set $iface up type can bitrate 1000000 2>/dev/null || true
done
sleep 0.5

echo "=== Auto-detecting arm CAN port ==="
CAN_PORT=""
for iface in can0 can1; do
    HAS_ERR=$(ip -details link show $iface 2>/dev/null | grep -c 'BUS-OFF\|NO-CARRIER' || true)
    if [ "$HAS_ERR" -gt 0 ]; then
        echo "  $iface: BUS-OFF/NO-CARRIER, skip"
        continue
    fi
    echo "  $iface: probing..."
    RESULT=$(timeout 8 python3 -c "
import time, sys
from pyAgxArm import create_agx_arm_config, AgxArmFactory, ArmModel
cfg = create_agx_arm_config(robot=ArmModel.NERO, comm='can', channel='$iface')
arm = AgxArmFactory.create_arm(cfg)
arm.connect()
start = time.time()
while time.time() - start < 3:
    if hasattr(arm, 'set_normal_mode'):
        arm.set_normal_mode()
    if arm.enable():
        a = arm.get_joint_angles()
        if a and a.msg:
            arm.disconnect()
            print('OK')
            sys.exit(0)
        break
    time.sleep(0.01)
arm.disconnect()
" 2>/dev/null || echo 'FAIL')
    if [ "$RESULT" = "OK" ]; then
        CAN_PORT=$iface
        echo "  $iface: arm found!"
        break
    else
        echo "  $iface: no arm (result=$RESULT)"
    fi
done

if [ -z "$CAN_PORT" ]; then
    echo "ERROR: cannot find arm on any CAN port"
    exit 1
fi

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
pkill -9 -f yolo_grasp 2>/dev/null || true
pkill -9 -f grasp_target_marker 2>/dev/null || true
pkill -9 -f robot_state_publisher 2>/dev/null || true
pkill -9 -f grasp_executor 2>/dev/null || true
pkill -9 -f grasp_train_driver 2>/dev/null || true
sleep 1

echo "=== Launching MoveIt + Arm ($CAN_PORT) in /arm ==="
ros2 launch agx_arm_ctrl start_single_agx_arm_moveit.launch.py \
  can_port:=$CAN_PORT \
  arm_type:=nero \
  effector_type:=agx_gripper \
  auto_enable:=true \
  auto_control_gate:=true \
  speed_percent:=10 \
  fw_version:=v111 \
  auto_home:=true \
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

echo "=== Starting RViz (in /arm via launch) ==="
ros2 launch agx_arm_moveit manual_rviz.launch.py \
  namespace:=arm \
  rviz_config:=/home/s1/tiaozhanbei/agx_arm_ros-ros2/src/config/yolo_config_armns.rviz &
RVIZ_PID=$!
echo "  rviz PID=$RVIZ_PID"

echo "=== Starting camera ==="
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

echo "=== Starting YOLO+Grasp with RL (in /arm) ==="
OMP_NUM_THREADS=2 ros2 run agx_arm_vision yolo_grasp \
  --ros-args -r __ns:=/arm \
  -p base_frame:=arm/base_link \
  -p camera_optical_frame:=arm/camera_color_optical_frame \
  -p rl_enable:=true &>/tmp/yolo.log &
YOLO_PID=$!
echo "  yolo_grasp PID=$YOLO_PID"

echo "=== Starting grasp target marker (in /arm) ==="
ros2 run agx_arm_vision grasp_target_marker \
  --ros-args -r __ns:=/arm &>/tmp/marker.log &
MARKER_PID=$!
echo "  marker PID=$MARKER_PID"

sleep 2

echo "=== Starting grasp executor (grasp_only, in /arm) ==="
ros2 run agx_arm_vision grasp_executor \
  --ros-args -r __ns:=/arm \
  -p base_link:=arm/base_link \
  -p end_effector_link:=arm/tcp_link \
  -p grasp_only:=true &>/tmp/grasp.log &
GRASP_PID=$!
echo "  grasp_executor PID=$GRASP_PID"

echo "=== Enabling RL training (if TRAIN=1) ==="
if [ "$TRAIN" = "1" ]; then
    for i in $(seq 1 30); do
        if ros2 service list 2>/dev/null | grep -q '/arm/grasp_rl/set_training'; then
            ros2 service call /arm/grasp_rl/set_training std_srvs/srv/SetBool "{data: true}"
            break
        fi
        echo "  waiting for grasp_rl service... ($i)"
        sleep 2
    done
else
    echo "  TRAIN=0: evaluation mode (greedy refine, no learning)"
fi

echo "=== Starting grasp train driver (if AUTO=1) ==="
AUTO_PID=""
if [ "$AUTO" = "1" ]; then
    ros2 run agx_arm_vision grasp_train_driver \
      --ros-args -r __ns:=/arm \
      -p max_attempts:=$MAX_ATTEMPTS &>/tmp/grasp_train.log &
    AUTO_PID=$!
    echo "  grasp_train_driver PID=$AUTO_PID"
else
    echo "  AUTO=0: manual trigger mode"
fi

echo "=== All started (grasp-only RL training stack, in /arm) ==="
echo "MoveIt=$MOVEIT_PID  RViz=$RVIZ_PID  Cam=$CAM_PID  YOLO=$YOLO_PID  Marker=$MARKER_PID  Grasp=$GRASP_PID  TrainDriver=${AUTO_PID:-off}"
echo ""
echo "Manual commands (注意 /arm/ 前缀):"
echo "  ros2 topic pub --once -w 1 /arm/manual_grasp_start std_msgs/msg/Empty '{}'   # AUTO=0 时手动触发一次抓取"
echo "  ros2 service call /arm/grasp_rl/set_training std_srvs/srv/SetBool \"{data: true}\"    # 开训练"
echo "  ros2 service call /arm/grasp_rl/set_training std_srvs/srv/SetBool \"{data: false}\"   # 关训练(冻结)"
echo "  ros2 service call /arm/grasp_rl/abort std_srvs/srv/Empty '{}'               # 人工中止(样本不入训练)"
echo ""
echo "数据: $RL_DATA_DIR/samples/   策略checkpoint: $RL_DATA_DIR/checkpoint/policy.pt"
echo "Logs: /tmp/yolo.log /tmp/grasp.log /tmp/grasp_train.log"
wait
