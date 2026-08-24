#!/bin/bash
# cleanup_stack.sh : 全清脚本 —— 杀掉所有机器人相关进程（机械臂栈 + 底盘栈）
# 用法: ./cleanup_stack.sh   （手动启动，不嵌入任何启动脚本）
#
# 关键：先杀"拉起进程的人"（start_chassis 的 while true 循环 / ros2 launch / ros2 run），
# 再杀子节点，避免 respawn；末尾多轮扫描清掉孤儿/刚重启的进程。
set -u

KILLED_ANY=""

kill_pat() {
    local pat="$1"
    if pkill -9 -f "$pat" 2>/dev/null; then
        KILLED_ANY=1
    fi
}

echo "=== [1] 先杀拉起进程的人 (launchers, 防 respawn) ==="
kill_pat 'start_chassis'
kill_pat 'ros2 launch'
kill_pat 'ros2 run'
kill_pat 'start_single_agx_arm_moveit'
kill_pat 'start_single_agx_arm'
kill_pat 'agx_arm_ctrl_single'
kill_pat 'teleop_web'
kill_pat 'http.server 8000'
sleep 1

echo "=== [2] 杀机械臂栈 (agx_arm) ==="
kill_pat 'yolo_grasp'
kill_pat 'place_planner'
kill_pat 'shelf_workflow'
kill_pat 'grasp_executor'
kill_pat 'grasp_target_marker'
kill_pat 'mock_gripper'
kill_pat 'camera_watchdog'
kill_pat 'realsense2_camera'
kill_pat 'aruco_tracker'
kill_pat 'virtual_aruco'
kill_pat 'virtual_yolo'
kill_pat 'virtual_depth_camera'
kill_pat 'robot_state_publisher'
kill_pat 'move_group'
kill_pat 'ros2_control_node'
kill_pat 'controller_manager'
kill_pat 'manual_arm_move'
kill_pat 'manual_gripper'
kill_pat 'tilt_measure'

echo "=== [3] 杀底盘栈 (chassis) ==="
kill_pat 'chassis_bridge'
kill_pat 'imu_node'
kill_pat 'wheel_odometry'
kill_pat 'ekf_node'
kill_pat 'ekf_filter'
kill_pat 'slam_toolbox'
kill_pat 'velodyne_driver'
kill_pat 'velodyne_pointcloud'
kill_pat 'velodyne_transform'
kill_pat 'vlp16_ring_scan'
kill_pat 'heading_correction'
kill_pat 'drift_compensation'
kill_pat 'kinematics_node'

echo "=== [4] 杀其它常驻 ==="
kill_pat 'map_watch'
kill_pat 'record_watch'
kill_pat 'rviz2'
kill_pat 'rosbridge_websocket'
kill_pat 'rosapi'

sleep 1

echo "=== [5] 多轮扫描，清 respawn/孤儿 (最多 3 轮) ==="
SWEEP_PAT="realsense|yolo_grasp|place_planner|shelf_workflow|grasp_executor|grasp_target_marker|mock_gripper|camera_watchdog|aruco_tracker|virtual_aruco|virtual_yolo|virtual_depth_camera|robot_state_publisher|move_group|ros2_control_node|controller_manager|agx_arm_ctrl|manual_arm|manual_gripper|tilt_measure|chassis_bridge|imu_node|wheel_odometry|ekf_node|slam_toolbox|velodyne|vlp16|heading_correction|drift_compensation|kinematics_node|start_chassis|ros2 launch|ros2 run|map_watch|record_watch|rviz2|rosbridge|rosapi"
for round in 1 2 3; do
    LEFT=$(ps aux | grep -iE "$SWEEP_PAT" | grep -v grep | grep -v "bash -c" | grep -v cleanup_stack | grep -v "ps aux")
    if [ -z "$LEFT" ]; then
        break
    fi
    echo "  round $round: 仍有残留，再清一次..."
    pkill -9 -f "$SWEEP_PAT" 2>/dev/null || true
    sleep 2
done

echo "=== 残留检查 ==="
LEFT=$(ps aux | grep -iE "$SWEEP_PAT" | grep -v grep | grep -v "bash -c" | grep -v cleanup_stack | grep -v "ps aux")
if [ -z "$LEFT" ]; then
    echo "全部清理完成，无残留。"
else
    echo "仍有残留:"
    echo "$LEFT" | awk '{printf "  %s\n", substr($0, index($0,$11), 60)}' | head -10
fi

echo "=== 当前内存 ==="
free -h | head -2
