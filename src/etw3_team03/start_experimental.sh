#!/usr/bin/env bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Robust workspace root detection
if [ -f "$SCRIPT_DIR/../../install/setup.bash" ]; then
    WS_DIR="$(cd "$SCRIPT_DIR/../.." && pwd)"
elif [ -f "$PWD/install/setup.bash" ]; then
    WS_DIR="$PWD"
elif [ -d "/home/etw3/etw3_ws" ]; then
    WS_DIR="/home/etw3/etw3_ws"
else
    WS_DIR="$(cd "$SCRIPT_DIR/../.." && pwd)"
fi

echo "========================================================"
echo "🏎️  Launching Experimental Autonomous Suite on Pi"
echo "📂 Workspace: $WS_DIR"
echo "========================================================"

# Trap SIGINT/EXIT to cleanly stop all background child processes & zero motors
trap 'kill $(jobs -p) 2>/dev/null; python3 -c "from freenove_driver.motor import Ordinary_Car; c=Ordinary_Car(); c.set_motor_model(0,0,0,0); c.close()" 2>/dev/null || true' EXIT

cd "$WS_DIR"
set +u
source /opt/ros/jazzy/setup.bash 2>/dev/null || source /opt/ros/humble/setup.bash 2>/dev/null || true
source /home/etw3/.etw3_camera_env 2>/dev/null || true
source "$WS_DIR/install/setup.bash" 2>/dev/null || true

# 1. Start camera_node if not already running
if ! ps aux | grep -i camera_node | grep -v grep > /dev/null; then
    echo "📷 Starting camera_node (640x480)..."
    ros2 run camera_ros camera_node --ros-args -p width:=640 -p height:=480 &
    sleep 2
else
    echo "📷 camera_node is already running."
fi

# 2. Start ultrasonic distance publisher if not already running
if ! ps aux | grep -i distance_publisher | grep -v grep > /dev/null; then
    echo "📏 Starting ultrasonic distance_publisher..."
    ros2 run sensor_nodes distance_publisher &
    sleep 1
else
    echo "📏 distance_publisher is already running."
fi

# 3. Start experimental dual-horizon lane detector
echo "👁️  Starting experimental dual-horizon lane detector..."
ros2 run vision_nodes experimental_lane_detector &
sleep 1

# 4. Start experimental adaptive lane follower
echo "🚗 Starting experimental adaptive lane follower..."
ros2 run safety_nodes experimental_lane_follower &

echo "========================================================"
echo "✅ All autonomous nodes active! Press Ctrl+C to stop."
echo "========================================================"
wait
