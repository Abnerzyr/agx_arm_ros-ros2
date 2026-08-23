#!/bin/bash
# start_arm_yolo_sim.sh : YOLO 视觉抓取（真机相机 + 仿真机械臂）
#
# 与 start_arm_yolo.sh 的唯一差异：机械臂不启用真机/CAN，改用
# MoveIt 仿真(FakeSystem) 在 RViz 内模拟运动；其余（真机相机、
# yolo_grasp、grasp_target_marker、grasp_executor、RViz、手动触发）
# 全部与真机流程保持一致。
#
# 需要真机 RealSense 相机已连接；未插相机时 realsense2_camera 会报错。
cd /home/s1/tiaozhanbei/agx_arm_ros-ros2
rm -f /dev/shm/fastrtps* 2>/dev/null
source install/setup.bash

echo "=== Killing old processes ==="
pkill -9 -f move_group 2>/dev/null || true
pkill -9 -f ros2_control_node 2>/dev/null || true
pkill -9 -f controller_manager 2>/dev/null || true
pkill -9 -f robot_state_publisher 2>/dev/null || true
pkill -9 -f rviz2 2>/dev/null || true
pkill -9 -f mock_gripper 2>/dev/null || true
pkill -9 -f realsense2_camera 2>/dev/null || true
pkill -9 -f yolo_grasp 2>/dev/null || true
pkill -9 -f place_planner 2>/dev/null || true
pkill -9 -f grasp_target_marker 2>/dev/null || true
pkill -9 -f grasp_executor 2>/dev/null || true
pkill -9 -f 'ros2 launch' 2>/dev/null || true
sleep 1

echo "=== Launching simulated arm (MoveIt + FakeSystem, no real arm) ==="
ros2 launch agx_arm_moveit demo.launch.py \
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

echo "=== Starting RViz ==="
rviz2 -d /home/s1/tiaozhanbei/agx_arm_ros-ros2/src/config/yolo_config.rviz &
RVIZ_PID=$!
echo "  rviz PID=$RVIZ_PID"

echo "=== Starting mock gripper ==="
ros2 run agx_arm_vision mock_gripper &>/tmp/mock_gripper.log &
MOCK_GRIPPER_PID=$!
echo "  mock_gripper PID=$MOCK_GRIPPER_PID"

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

echo "=== Starting YOLO+Grasp ==="
ros2 run agx_arm_vision yolo_grasp &>/tmp/yolo.log &
YOLO_PID=$!
echo "  yolo_grasp PID=$YOLO_PID"

echo "=== Starting place planner ==="
ros2 run agx_arm_vision place_planner &>/tmp/place.log &
PLACE_PID=$!
echo "  place_planner PID=$PLACE_PID"

echo "=== Starting grasp target marker ==="
ros2 run agx_arm_vision grasp_target_marker &>/tmp/marker.log &
MARKER_PID=$!
echo "  marker PID=$MARKER_PID"

sleep 2

echo "=== Starting grasp executor ==="
# 仿真关节反馈在 /control/joint_states（真机为 /feedback/joint_states），
# 其余逻辑与真机完全一致。
ros2 run agx_arm_vision grasp_executor \
  --ros-args -r /feedback/joint_states:=/control/joint_states \
  &>/tmp/grasp.log &
GRASP_PID=$!
echo "  grasp_executor PID=$GRASP_PID"

echo "=== All started (YOLO + simulated arm) ==="
echo "MoveIt=$MOVEIT_PID  MockGripper=$MOCK_GRIPPER_PID  Cam=$CAM_PID  YOLO=$YOLO_PID  Place=$PLACE_PID  Marker=$MARKER_PID  Grasp=$GRASP_PID"
echo ""
echo "Manual commands:"
echo "  ros2 topic pub --once -w 1 /manual_grasp_start std_msgs/msg/Empty '{}'"
echo "  ros2 topic pub --once -w 1 /manual_release std_msgs/msg/Empty '{}'"
wait
