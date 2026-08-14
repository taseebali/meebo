#!/usr/bin/env bash

echo "========================================================="
echo " 🛑 ETW3 TEAM 03 - MASTER KILL & CLEANUP SCRIPT 🛑"
echo "========================================================="

# 1. Safely stop motors first
echo "1. Zeroing motor speeds and releasing I2C bus..."
python3 -c "
try:
    from freenove_driver.motor import Ordinary_Car
    car = Ordinary_Car()
    car.set_motor_model(0, 0, 0, 0)
    car.close()
    print('   ✅ Motors safely stopped.')
except Exception as e:
    print('   ⚠️ Motor note:', e)
" 2>/dev/null || true

# 2. Terminate all ROS 2 & Python nodes, camera, bridges, and bags
echo "2. Killing all running nodes, camera processes & bridges..."

PROCS=(
    "camera_node"
    "camera_ros"
    "distance_publisher"
    "distance_watch"
    "experimental_lane_follower"
    "experimental_lane_detector"
    "lane_follower"
    "lane_offset_publisher"
    "estop_node"
    "frame_saver"
    "cmd_vel_bridge"
    "web_teleop_node"
    "foxglove_bridge"
    "ros2 bag"
    "tune_threshold"
    "sample_hsv"
)

for proc in "${PROCS[@]}"; do
    if pgrep -f "$proc" > /dev/null; then
        echo "   -> Stopping $proc..."
        pkill -9 -f "$proc" 2>/dev/null || true
    fi
done

# 3. Stop ROS 2 daemon to clear stale topic/node graphs
echo "3. Restarting ROS 2 daemon..."
set +u
source /opt/ros/jazzy/setup.bash 2>/dev/null || source /opt/ros/humble/setup.bash 2>/dev/null || true
ros2 daemon stop >/dev/null 2>&1 || true
sleep 1
ros2 daemon start >/dev/null 2>&1 || true

echo "========================================================="
echo " ✅ CLEANUP COMPLETE! All hardware & nodes released."
echo " You can now safely start fresh nodes or camera_node."
echo "========================================================="
