#!/bin/bash
# start_aruco_align_armns.sh : 仅 Aruco 码识别 + 机械臂对准流程（真实相机 + 仿真臂，/arm 版）
#
# 最小化流程：只做 shelf 全流程里的"对准"环节，用于快速验证
# aruco 码识别与机械臂对准（ALIGN）逻辑，不启动 yolo_grasp / place_planner。
#
# 节点组成（真实相机 + 仿真臂，臂进 /arm 命名空间）：
#   - MoveIt FakeSystem (demo.launch.py, namespace:=arm)
#   - RViz (manual_rviz.launch.py, /arm) —— 看臂模型与 TF
#   - mock_gripper + grasp_executor —— shelf_workflow 依赖 executor 的
#     /arm/grasp_executor_state 判断"空闲才允许移动"（grasp_executor 不抓取，纯 idle）
#   - RealSense 相机（全局 /camera/...）+ aruco_tracker（全局 /aruco_detections）
#   - shelf_workflow（/arm）—— HOME_CHECK → NOMINAL_POSE → ALIGN
#
# 流程：发 /arm/task_command 后，机械臂回 home → 移到名义观察位姿 →
#       aruco 精对准（日志出现 "Aligned to marker" 即对准成功）。
#       之后进入 WAIT_DETECT（无 yolo，只会周期性打警告，流程停住），
#       用于观察对准结果；重测再发一次 /arm/task_command。
#
# 前提：
#   - 真机 RealSense 相机已连接
#   - 真实 aruco 码(4X4_50) 摆放在 shelf_layers_sim.yaml 对应位置附近
#     （层1 约为 base_link 系 (-0.40, -0.22, 0.30)，若出视野 ALIGN 会卡住后放弃）
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

sleep 2

echo "=== Starting grasp executor (in /arm, idle only) ==="
# shelf_workflow 在 NOMINAL_POSE / ALIGN 前会检查 executor 是否 IDLE；
# 本脚本不抓取，executor 只负责发布 /arm/grasp_executor_state=IDLE。
# 仿真关节反馈在 /arm/control/joint_states，remap 用相对写法。
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

echo "=== All started (aruco align only, real camera + simulated arm, in /arm) ==="
echo "MoveIt=$MOVEIT_PID  RViz=$RVIZ_PID  MockGripper=$MOCK_GRIPPER_PID  Cam=$CAM_PID  ArUco=$ARUCO_PID  Grasp=$GRASP_PID  Shelf=$SHELF_PID"
echo ""
echo "Manual commands (注意 /arm/ 前缀):"
echo "  ros2 topic pub --once -w 1 /arm/task_command std_msgs/msg/Int32 '{data: 1}'"
echo ""
echo "看对准结果: tail -f /tmp/shelf.log   (出现 'Aligned to marker' 即对准成功)"
echo "Logs: /tmp/shelf.log /tmp/grasp.log /tmp/aruco.log /tmp/mock_gripper.log"
wait
