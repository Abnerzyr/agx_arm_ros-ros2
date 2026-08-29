#!/bin/bash
# start_shelf_realcam.sh : 货架分层抓取-放置（真机相机 + 仿真机械臂，臂进 /arm 命名空间版）
#
# 与底盘同时常驻时使用；臂的 TF/话题带 /arm 前缀，避免 base_link 冲突
# （与 start_shelf.sh / start_yolo_armns.sh / start_shelf_sim.sh 同一套机制）。
#
# 与 start_shelf_sim.sh（纯仿真）的区别：视觉全部用真机——RealSense 相机、
# 真实 aruco_tracker、真实 yolo_grasp、真实 place_planner；机械臂仍用
# MoveIt 仿真(FakeSystem)。用于交叉验证：真视觉 vs 虚拟视觉 在同一个
# mock 臂 + MoveIt 规划下的差异，定位问题出在视觉侧还是运动/规划侧。
#
# 相机(/camera/...) 与 aruco(/aruco_detections) 保持全局绝对话题；其余进 /arm。
# 手动触发话题带 /arm/ 前缀。
#
# 前提：
#   - 真机 RealSense 相机已连接
#   - 相机前放一个 yolo 可检出的物体（默认类别: rubik's cube）
#   - 真实 aruco 码(4X4_50) 摆放在 shelf_layers_sim.yaml 对应位置附近
#     （层1 约为 base_link 系 (-0.40, -0.22, 0.30)，若出视野 ALIGN 会卡住）
cd /home/s1/tiaozhanbei/agx_arm_ros-ros2
rm -f /dev/shm/fastrtps* 2>/dev/null
source install/setup.bash

echo "=== Killing old processes ==="
pkill -9 -f agx_arm_ctrl_single 2>/dev/null || true
pkill -9 -f start_single_agx_arm_moveit 2>/dev/null || true
pkill -9 -f move_group 2>/dev/null || true
pkill -9 -f ros2_control_node 2>/dev/null || true
pkill -9 -f controller_manager 2>/dev/null || true
pkill -9 -f robot_state_publisher 2>/dev/null || true
pkill -9 -f rviz2 2>/dev/null || true
pkill -9 -f mock_gripper 2>/dev/null || true
pkill -9 -f realsense2_camera 2>/dev/null || true
pkill -9 -f aruco_tracker 2>/dev/null || true
pkill -9 -f yolo_grasp 2>/dev/null || true
pkill -9 -f place_planner 2>/dev/null || true
pkill -9 -f grasp_target_marker 2>/dev/null || true
pkill -9 -f grasp_executor 2>/dev/null || true
pkill -9 -f shelf_workflow 2>/dev/null || true
pkill -9 -f 'ros2 launch' 2>/dev/null || true
sleep 1

echo "=== Launching simulated arm (MoveIt + FakeSystem, in /arm, no real arm) ==="
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
  rviz_config:=/home/s1/tiaozhanbei/agx_arm_ros-ros2/src/agx_arm_vision/rviz/shelf_realcam_armns.rviz &
RVIZ_PID=$!
echo "  rviz PID=$RVIZ_PID"

echo "=== Starting mock gripper (in /arm) ==="
ros2 run agx_arm_vision mock_gripper \
  --ros-args -r __ns:=/arm &>/tmp/mock_gripper.log &
MOCK_GRIPPER_PID=$!
echo "  mock_gripper PID=$MOCK_GRIPPER_PID"

echo "=== Starting camera (real RealSense, global) ==="
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

echo "=== Starting ArUco tracker (real, global) ==="
ros2 run aruco_opencv aruco_tracker_autostart \
  --ros-args \
  -p cam_base_topic:=/camera/camera/color/image_raw \
  -p marker_dict:=4X4_50 \
  -p marker_size:=0.05 &>/tmp/aruco.log &
ARUCO_PID=$!
echo "  aruco_tracker PID=$ARUCO_PID"

echo "=== Starting YOLO+Grasp (real, in /arm) ==="
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

echo "=== All started (real camera + simulated arm + shelf workflow, in /arm) ==="
echo "MoveIt=$MOVEIT_PID  RViz=$RVIZ_PID  MockGripper=$MOCK_GRIPPER_PID  Cam=$CAM_PID  ArUco=$ARUCO_PID  YOLO=$YOLO_PID  Place=$PLACE_PID  Marker=$MARKER_PID  Grasp=$GRASP_PID  Shelf=$SHELF_PID"
echo ""
echo "Manual commands (注意 /arm/ 前缀):"
echo "  ros2 topic pub --once -w 1 /arm/shelf/skip_align std_msgs/msg/Empty '{}'  # 无aruco时跳过对准(测试)"
echo "  ros2 topic pub --once -w 1 /arm/task_command std_msgs/msg/Int32 '{data: 1}'"
echo "  ros2 topic pub --once -w 1 /arm/release_command std_msgs/msg/Empty '{}'"
echo "  ros2 topic pub --once -w 1 /arm/manual_release_force std_msgs/msg/Empty '{}'  # 原地松爪（手动兜底）"
echo ""
echo "Logs: /tmp/shelf.log /tmp/grasp.log /tmp/yolo.log /tmp/aruco.log /tmp/place.log"
wait
