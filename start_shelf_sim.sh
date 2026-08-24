#!/bin/bash
# start_shelf_sim.sh : 货架分层抓取-放置工作流【纯仿真测试，臂进 /arm 命名空间版】
#
# 与底盘同时常驻时使用；臂的 TF/话题带 /arm 前缀，避免 base_link 冲突
# （与 start_shelf.sh / start_yolo_armns.sh 同一套命名空间机制）。
#
# 不使用真机机械臂，也不使用真机相机：
#   - 机械臂: MoveIt FakeSystem (demo.launch.py, namespace:=arm) 在 RViz 内模拟运动
#   - aruco: virtual_aruco_pub 发布模拟 marker (位置=该层货架处) —— 保持全局绝对 /aruco_detections
#   - 检测: virtual_yolo_target 发布模拟 grasp_pose 与 yolo/target_box —— 进 /arm
#   - 归位: shelf_workflow 直接 MoveIt move_to_joints 回 home (同 grasp_executor)
#   - 放置: place_planner 基于 virtual_depth_camera 的模拟深度图选点
#
# 手动触发话题带 /arm/ 前缀（如 /arm/task_command）。
# LAYER 环境变量选择层号 (1/2/3)，默认 1。
cd /home/s1/tiaozhanbei/agx_arm_ros-ros2
rm -f /dev/shm/fastrtps* 2>/dev/null
source install/setup.bash

LAYER=${LAYER:-1}
case $LAYER in
  1) ARUCO_Z=0.30; TARGET_Z=0.365; BOX_Z=0.33 ;;
  2) ARUCO_Z=0.60; TARGET_Z=0.665; BOX_Z=0.63 ;;
  3) ARUCO_Z=0.90; TARGET_Z=0.965; BOX_Z=0.93 ;;
  *) echo "ERROR: LAYER must be 1/2/3"; exit 1 ;;
esac

echo "=== Layer $LAYER (aruco z=$ARUCO_Z, target z=$TARGET_Z) ==="

echo "=== Killing old processes ==="
pkill -9 -f agx_arm_ctrl_single 2>/dev/null || true
pkill -9 -f start_single_agx_arm_moveit 2>/dev/null || true
pkill -9 -f move_group 2>/dev/null || true
pkill -9 -f ros2_control_node 2>/dev/null || true
pkill -9 -f controller_manager 2>/dev/null || true
pkill -9 -f robot_state_publisher 2>/dev/null || true
pkill -9 -f rviz2 2>/dev/null || true
pkill -9 -f mock_gripper 2>/dev/null || true
pkill -9 -f virtual_aruco_pub 2>/dev/null || true
pkill -9 -f virtual_depth_camera 2>/dev/null || true
pkill -9 -f virtual_yolo_target 2>/dev/null || true
pkill -9 -f place_planner 2>/dev/null || true
pkill -9 -f grasp_target_marker 2>/dev/null || true
pkill -9 -f grasp_executor 2>/dev/null || true
pkill -9 -f shelf_workflow 2>/dev/null || true
pkill -9 -f 'ros2 launch' 2>/dev/null || true
sleep 1

echo "=== Launching simulated arm (MoveIt FakeSystem, in /arm, no real arm) ==="
ros2 launch agx_arm_moveit demo.launch.py \
  namespace:=arm \
  arm_type:=nero \
  effector_type:=agx_gripper \
  follow:=false \
  use_rviz:=false &
MOVEIT_PID=$!

echo "=== Waiting for move_group ... ==="
for i in $(seq 1 15); do
    if ros2 action list 2>/dev/null | grep -q move_action; then
        echo "  move_group ready"
        break
    fi
    echo "  waiting for move_group... ($i)"
    sleep 2
done

echo "=== Starting RViz (in /arm via launch) ==="
ros2 launch agx_arm_moveit manual_rviz.launch.py \
  namespace:=arm \
  rviz_config:=/home/s1/tiaozhanbei/agx_arm_ros-ros2/src/agx_arm_vision/rviz/shelf_sim_armns.rviz &
RVIZ_PID=$!
echo "  rviz PID=$RVIZ_PID"

echo "=== Starting mock gripper (in /arm) ==="
ros2 run agx_arm_vision mock_gripper \
  --ros-args -r __ns:=/arm &>/tmp/mock_gripper.log &
MOCK_GRIPPER_PID=$!
echo "  mock_gripper PID=$MOCK_GRIPPER_PID"

echo "=== Starting virtual depth camera (global; camera topics stay absolute) ==="
ros2 run agx_arm_vision virtual_depth_camera \
  --ros-args \
  -p base_frame:=arm/base_link \
  -p camera_frame:=arm/camera_color_optical_frame &>/tmp/vdepth.log &
VDEPTH_PID=$!
echo "  virtual_depth_camera PID=$VDEPTH_PID"

echo "=== Starting virtual aruco (global; /aruco_detections stays absolute, marker_id=$LAYER) ==="
ros2 run agx_arm_vision virtual_aruco_pub \
  --ros-args \
  -p marker_id:=$LAYER \
  -p x:=-0.40 \
  -p y:=-0.22 \
  -p z:=$ARUCO_Z \
  -p base_frame:=arm/base_link \
  -p camera_frame:=arm/camera_color_optical_frame &>/tmp/varuco.log &
VARUCO_PID=$!
echo "  virtual_aruco_pub PID=$VARUCO_PID"

echo "=== Starting virtual yolo target (in /arm, grasp point on layer $LAYER) ==="
ros2 run agx_arm_vision virtual_yolo_target \
  --ros-args -r __ns:=/arm \
  -p x:=-0.47 \
  -p y:=-0.16 \
  -p z:=$TARGET_Z \
  -p box_center_z:=$BOX_Z \
  -p box_size:=0.001 \
  -p grasp_yaw_deg:=90.0 \
  -p base_frame:=arm/base_link &>/tmp/vyolo.log &
VYOLO_PID=$!
echo "  virtual_yolo_target PID=$VYOLO_PID"

echo "=== Starting place planner (in /arm) ==="
OPENBLAS_NUM_THREADS=2 ros2 run agx_arm_vision place_planner \
  --ros-args -r __ns:=/arm \
  -p base_frame:=arm/base_link \
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
# 仿真关节反馈在 /arm/control/joint_states（真机为 /feedback/joint_states）；
# remap 用相对写法，命名空间下自动解析为 /arm/feedback/joint_states := /arm/control/joint_states
ros2 run agx_arm_vision grasp_executor \
  --ros-args -r __ns:=/arm \
  -r feedback/joint_states:=control/joint_states \
  -p base_link:=arm/base_link \
  -p end_effector_link:=arm/tcp_link &>/tmp/grasp.log &
GRASP_PID=$!
echo "  grasp_executor PID=$GRASP_PID"

echo "=== Starting shelf workflow (in /arm, sim config) ==="
ros2 run agx_arm_vision shelf_workflow \
  --ros-args -r __ns:=/arm \
  -p base_frame:=arm/base_link \
  -p end_effector_link:=arm/tcp_link \
  -p camera_frame:=arm/camera_color_optical_frame \
  -p config_file:=/home/s1/tiaozhanbei/agx_arm_ros-ros2/src/agx_arm_vision/config/shelf_layers_sim.yaml \
  &>/tmp/shelf.log &
SHELF_PID=$!
echo "  shelf_workflow PID=$SHELF_PID"

echo "=== All started (YOLO + shelf workflow, pure simulation, in /arm) ==="
echo "MoveIt=$MOVEIT_PID  RViz=$RVIZ_PID  MockGripper=$MOCK_GRIPPER_PID  VDepth=$VDEPTH_PID  VAruco=$VARUCO_PID  VYolo=$VYOLO_PID  Place=$PLACE_PID  Marker=$MARKER_PID  Grasp=$GRASP_PID  Shelf=$SHELF_PID"
echo ""
echo "Manual commands (注意 /arm/ 前缀):"
echo "  ros2 topic pub --once -w 1 /arm/task_command std_msgs/msg/Int32 '{data: $LAYER}'"
echo "  ros2 topic pub --once -w 1 /arm/release_command std_msgs/msg/Empty '{}'"
echo ""
echo "Logs: /tmp/shelf.log /tmp/grasp.log /tmp/place.log /tmp/varuco.log /tmp/vyolo.log"
wait
