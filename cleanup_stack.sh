#!/bin/bash
# cleanup_stack.sh: stop all robot ROS services and manually started nodes.
# Usage: ./cleanup_stack.sh
#
# systemd services must be stopped before killing individual processes.
# Otherwise Restart=always brings the navigation stack back after a few seconds.
set -u

KILLED_ANY=""
ROBOT_SERVICES=(
    task-scheduler.service
    record-watcher.service
    map-watch.service
    chassis.service
)

if [ "$(id -u)" -eq 0 ]; then
    SYSTEMCTL=(systemctl)
else
    echo "=== Requesting permission to stop robot system services ==="
    if ! sudo -v; then
        echo "Unable to obtain sudo permission; aborting full cleanup."
        exit 1
    fi
    SYSTEMCTL=(sudo systemctl)
fi

kill_pat() {
    local pat="$1"
    if pkill -9 -f "$pat" 2>/dev/null; then
        KILLED_ANY=1
    fi
}

stop_unit() {
    local unit="$1"
    if ! systemctl cat "$unit" >/dev/null 2>&1; then
        return
    fi
    if ! systemctl is-active --quiet "$unit"; then
        echo "  $unit already stopped"
        return
    fi

    echo "  stopping $unit"
    if timeout 40 "${SYSTEMCTL[@]}" stop "$unit"; then
        return
    fi

    echo "  $unit did not stop cleanly; killing its entire cgroup"
    "${SYSTEMCTL[@]}" kill --kill-who=all --signal=SIGKILL \
        "$unit" 2>/dev/null || true
    "${SYSTEMCTL[@]}" stop "$unit" 2>/dev/null || true
}

echo "=== [1] Stop systemd-managed robot stacks ==="
for unit in "${ROBOT_SERVICES[@]}"; do
    stop_unit "$unit"
done
sleep 2

echo "=== [2] Stop remaining launchers to prevent respawn ==="
kill_pat 'start_chassis'
kill_pat 'start_chassis_auto'
kill_pat 'start_task_scheduler'
kill_pat 'ros2 launch'
kill_pat 'ros2 run'
kill_pat 'start_single_agx_arm_moveit'
kill_pat 'start_single_agx_arm'
kill_pat 'agx_arm_ctrl_single'
kill_pat 'teleop_web'
kill_pat 'http.server 8000'
sleep 1

echo "=== [3] Stop arm and vision stack ==="
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

echo "=== [4] Stop chassis, lidar and Nav2 stack ==="
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
kill_pat 'pointcloud_to_laserscan'
kill_pat 'range_filter'
kill_pat 'heading_correction'
kill_pat 'drift_compensation'
kill_pat 'kinematics_node'
kill_pat 'map_server'
kill_pat 'amcl'
kill_pat 'controller_server'
kill_pat 'smoother_server'
kill_pat 'planner_server'
kill_pat 'behavior_server'
kill_pat 'bt_navigator'
kill_pat 'waypoint_follower'
kill_pat 'velocity_smoother'
kill_pat 'lifecycle_manager'

echo "=== [5] Stop schedulers, watchers and recording ==="
kill_pat 'competition_node'
kill_pat 'arm_task_bridge'
kill_pat 'arm_worker_stub'
kill_pat 'task_scheduler'
kill_pat 'map_watch'
kill_pat 'record_watch'
kill_pat 'ros2 bag record'
kill_pat 'rviz2'
kill_pat 'rosbridge_websocket'
kill_pat 'rosapi_node'

sleep 2

SWEEP_PAT="realsense|yolo_grasp|place_planner|shelf_workflow|grasp_executor"
SWEEP_PAT+="|grasp_target_marker|mock_gripper|camera_watchdog|aruco_tracker"
SWEEP_PAT+="|virtual_aruco|virtual_yolo|virtual_depth_camera|robot_state_publisher"
SWEEP_PAT+="|move_group|ros2_control_node|controller_manager|agx_arm_ctrl"
SWEEP_PAT+="|manual_arm|manual_gripper|tilt_measure|chassis_bridge|imu_node"
SWEEP_PAT+="|wheel_odometry|ekf_node|ekf_filter|slam_toolbox|velodyne|vlp16"
SWEEP_PAT+="|pointcloud_to_laserscan|range_filter|heading_correction"
SWEEP_PAT+="|drift_compensation|kinematics_node|map_server|amcl"
SWEEP_PAT+="|controller_server|smoother_server|planner_server|behavior_server"
SWEEP_PAT+="|bt_navigator|waypoint_follower|velocity_smoother|lifecycle_manager"
SWEEP_PAT+="|competition_node|arm_task_bridge|arm_worker_stub|task_scheduler"
SWEEP_PAT+="|start_chassis|start_task_scheduler|ros2 launch|ros2 run|map_watch"
SWEEP_PAT+="|record_watch|ros2 bag record|rviz2|rosbridge|rosapi"

echo "=== [6] Sweep respawned and orphaned processes ==="
for round in 1 2 3; do
    LEFT=$(pgrep -af "$SWEEP_PAT" 2>/dev/null || true)
    if [ -z "$LEFT" ]; then
        break
    fi
    echo "  round $round: residual processes found; cleaning again"
    pkill -9 -f "$SWEEP_PAT" 2>/dev/null || true
    sleep 2
done

if command -v ros2 >/dev/null 2>&1; then
    ros2 daemon stop >/dev/null 2>&1 || true
fi

echo "=== Final verification ==="
FAILED=""
for unit in "${ROBOT_SERVICES[@]}"; do
    if systemctl is-active --quiet "$unit"; then
        echo "  service still active: $unit"
        FAILED=1
    fi
done

LEFT=$(pgrep -af "$SWEEP_PAT" 2>/dev/null || true)
if [ -n "$LEFT" ]; then
    echo "  residual robot processes:"
    while IFS= read -r line; do
        echo "    ${line:0:100}"
    done <<< "$LEFT"
    FAILED=1
fi

if [ -z "$FAILED" ]; then
    echo "Full cleanup complete: no robot services or processes remain."
else
    echo "Full cleanup incomplete; inspect the entries above."
    exit 1
fi

echo "=== Current memory ==="
free -h
