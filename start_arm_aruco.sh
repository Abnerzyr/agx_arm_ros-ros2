#!/bin/bash
cd /home/s1/tiaozhanbei/agx_arm_ros-ros2
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

echo "=== Killing old processes ==="
pkill -f start_single_agx_arm_moveit 2>/dev/null || true
pkill -9 -f move_group 2>/dev/null || true
pkill -9 -f ros2_control_node 2>/dev/null || true
pkill -9 -f rviz2 2>/dev/null || true
pkill -9 -f agx_arm_ctrl_single 2>/dev/null || true
pkill -9 -f realsense2_camera 2>/dev/null || true
pkill -9 -f aruco_tracker 2>/dev/null || true
pkill -9 -f pointcloud_grasp 2>/dev/null || true
pkill -9 -f grasp_executor 2>/dev/null || true
pkill -9 -f vision_grasp_node 2>/dev/null || true
pkill -9 -f ggcnn_grasp 2>/dev/null || true
sleep 1

echo "=== Launching MoveIt + Arm ($CAN_PORT) ==="
ros2 launch agx_arm_ctrl start_single_agx_arm_moveit.launch.py \
  can_port:=$CAN_PORT \
  arm_type:=nero \
  effector_type:=agx_gripper \
  auto_enable:=true \
  auto_control_gate:=true \
  speed_percent:=5 \
  fw_version:=v111 \
  auto_home:=true \
  follow:=true &
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

echo "=== Starting camera ==="
ros2 run realsense2_camera realsense2_camera_node \
  --ros-args -r __node:=camera -r __ns:=/camera \
  -p align_depth.enable:=true -p publish_tf:=false &
CAM_PID=$!

sleep 3

echo "=== Starting ArUco ==="
ros2 run aruco_opencv aruco_tracker_autostart \
  --ros-args \
  -p cam_base_topic:=/camera/camera/color/image_raw \
  -p marker_dict:=4X4_50 \
  -p marker_size:=0.05 &
ARUCO_PID=$!

sleep 3

echo "=== All started (ArUco) ==="
echo "CAN=$CAN_PORT  MoveIt=$MOVEIT_PID  Cam=$CAM_PID  ArUco=$ARUCO_PID"
echo ""
echo "Manual commands:"
echo "  ros2 run agx_arm_vision vision_grasp_node --ros-args -p target_marker_id:=-1 &>/tmp/aruco.log &"
echo "  ros2 run agx_arm_vision grasp_executor --ros-args -p force_threshold:=1.5 -p constrain_orientation:=true &>/tmp/grasp.log &"
echo "  ros2 service call /move_home std_srvs/srv/Empty"
wait
