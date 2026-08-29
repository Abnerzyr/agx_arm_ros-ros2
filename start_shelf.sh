#!/bin/bash
# start_shelf.sh : 货架分层抓取-放置完整工作流（真机，臂进 /arm 命名空间版）
# 臂与底盘同时常驻时使用；臂的 TF/话题带 /arm 前缀，避免 base_link 冲突。
#
# 流程: 归位 → /arm/task_command(层号) → 粗对准该层 → aruco 精对准 →
#       yolo 检测该层物体 → /arm/manual_grasp_start 抓取 → 回初始位 →
#       /arm/release_command → place_planner 选点放置到眼前平面
#
# 与 start_arm_yolo.sh 的差异: 额外启动 aruco_tracker 与 shelf_workflow 编排节点。
cd /home/s1/tiaozhanbei/agx_arm_ros-ros2
rm -f /dev/shm/fastrtps* 2>/dev/null
source install/setup.bash

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
pkill -9 -f aruco_tracker 2>/dev/null || true
pkill -9 -f yolo_grasp 2>/dev/null || true
pkill -9 -f place_planner 2>/dev/null || true
pkill -9 -f grasp_target_marker 2>/dev/null || true
pkill -9 -f robot_state_publisher 2>/dev/null || true
pkill -9 -f shelf_workflow 2>/dev/null || true
sleep 1
pkill -9 -f grasp_executor 2>/dev/null || true
pkill -9 -f vision_grasp_node 2>/dev/null || true
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

echo "=== Starting ArUco tracker ==="
ros2 run aruco_opencv aruco_tracker_autostart \
  --ros-args \
  -p cam_base_topic:=/camera/camera/color/image_raw \
  -p marker_dict:=4X4_50 \
  -p marker_size:=0.05 &>/tmp/aruco.log &
ARUCO_PID=$!
echo "  aruco_tracker PID=$ARUCO_PID"

echo "=== Starting YOLO+Grasp (in /arm) ==="
OMP_NUM_THREADS=2 ros2 run agx_arm_vision yolo_grasp \
  --ros-args -r __ns:=/arm \
  -p base_frame:=arm/base_link \
  -p camera_optical_frame:=arm/camera_color_optical_frame &>/tmp/yolo.log &
YOLO_PID=$!
echo "  yolo_grasp PID=$YOLO_PID"

echo "=== Starting place planner (in /arm) ==="
OPENBLAS_NUM_THREADS=2 ros2 run agx_arm_vision place_planner \
  --ros-args -r __ns:=/arm \
  -p base_frame:=arm/base_link \
  -p end_effector_link:=arm/tcp_link \
  -p camera_optical_frame:=arm/camera_color_optical_frame \
  -p process_period:=1.0 &>/tmp/place.log &
PLACE_PID=$!
echo "  place_planner PID=$PLACE_PID"

echo "=== Starting grasp target marker (in /arm) ==="
ros2 run agx_arm_vision grasp_target_marker \
  --ros-args -r __ns:=/arm &>/tmp/marker.log &
MARKER_PID=$!
echo "  marker PID=$MARKER_PID"

sleep 2

echo "=== Starting grasp executor (in /arm) ==="
ros2 run agx_arm_vision grasp_executor \
  --ros-args -r __ns:=/arm \
  -p base_link:=arm/base_link \
  -p end_effector_link:=arm/tcp_link &>/tmp/grasp.log &
GRASP_PID=$!
echo "  grasp_executor PID=$GRASP_PID"

echo "=== Starting shelf workflow (in /arm) ==="
ros2 run agx_arm_vision shelf_workflow \
  --ros-args -r __ns:=/arm \
  -p base_frame:=arm/base_link \
  -p end_effector_link:=arm/tcp_link \
  -p camera_frame:=arm/camera_color_optical_frame &>/tmp/shelf.log &
SHELF_PID=$!
echo "  shelf_workflow PID=$SHELF_PID"

echo "=== All started (YOLO + shelf workflow) ==="
echo "CAN=$CAN_PORT  MoveIt=$MOVEIT_PID  Cam=$CAM_PID  ArUco=$ARUCO_PID  YOLO=$YOLO_PID  Place=$PLACE_PID  Marker=$MARKER_PID  Grasp=$GRASP_PID  Shelf=$SHELF_PID"
echo ""
echo "Manual commands (注意 /arm/ 前缀):"
echo "  ros2 topic pub --once -w 1 /arm/shelf/skip_align std_msgs/msg/Empty '{}'   # 无aruco时跳过对准(测试)"
echo "  ros2 topic pub --once -w 1 /arm/shelf/preset_home std_msgs/msg/Empty '{}' # NOMINAL_POSE 用 home 预设位(测试)"
echo "  ros2 topic pub --once -w 1 /arm/task_command std_msgs/msg/Int32 '{data: 1}'"
echo "  ros2 topic pub --once -w 1 /arm/release_command std_msgs/msg/Empty '{}'"
echo "  ros2 topic pub --once -w 1 /arm/manual_release_force std_msgs/msg/Empty '{}'  # 原地松爪（手动兜底）"
wait
