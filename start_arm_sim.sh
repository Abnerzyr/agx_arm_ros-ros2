#!/bin/bash
cd /home/s1/tiaozhanbei/agx_arm_ros-ros2
rm -f /dev/shm/fastrtps_port* 2>/dev/null
source install/setup.bash

SIMULATE_OBJECT="${1:-true}"

echo "=== Killing old sim processes ==="
pkill -9 -f move_group 2>/dev/null || true
pkill -9 -f ros2_control_node 2>/dev/null || true
pkill -9 -f robot_state_publisher 2>/dev/null || true
pkill -9 -f moveit_simple_controller_manager 2>/dev/null || true
pkill -9 -f rviz2 2>/dev/null || true
pkill -9 -f mock_gripper 2>/dev/null || true
pkill -9 -f grasp_executor 2>/dev/null || true
pkill -9 -f grasp_target_marker 2>/dev/null || true
pkill -9 -f 'ros2 launch' 2>/dev/null || true
sleep 1

echo "=== Launching simulation (mock arm + mock gripper + grasp_executor) ==="
ros2 launch agx_arm_vision grasp_executor_sim.launch.py \
  simulate_object:=$SIMULATE_OBJECT &
LAUNCH_PID=$!

echo "=== Waiting for move_group ==="
for i in $(seq 1 15); do
    if ros2 action list 2>/dev/null | grep -q move_action; then
        echo "  move_group ready"
        break
    fi
    echo "  waiting for move_group... ($i)"
    sleep 2
done

echo "=== All started (sim, simulate_object=$SIMULATE_OBJECT) ==="
echo ""
echo "Test commands:"
echo "  ros2 topic pub --once /grasp_pose geometry_msgs/msg/PoseStamped '{header: {frame_id: \"base_link\"}, pose: {position: {x: 0.4, y: 0.0, z: 0.3}, orientation: {w: 1.0}}}'"
echo "  ros2 topic pub --once /manual_grasp_start std_msgs/msg/Empty"
echo "  ros2 topic pub --once /manual_release std_msgs/msg/Empty"
echo "  ros2 topic echo /feedback/gripper_status"
wait
