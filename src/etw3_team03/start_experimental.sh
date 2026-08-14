#!/usr/bin/env bash
# ========================================================
# 🏎️ Master Autonomous Launcher for Experimental Suite
# ========================================================

# SCRIPT_DIR is src/etw3_team03/
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

# Trap SIGINT, SIGTERM, EXIT to cleanly stop child processes & zero motors
cleanup() {
    echo ""
    echo "🛑 Shutting down all autonomous nodes..."
    kill $(jobs -p) 2>/dev/null || true
    pkill -9 -f experimental_lane_follower 2>/dev/null || true
    pkill -9 -f experimental_lane_detector 2>/dev/null || true
    # Zero motors safely on I2C bus
    python3 -c "
try:
    from freenove_driver.motor import Ordinary_Car
    c = Ordinary_Car()
    c.set_motor_model(0, 0, 0, 0)
    c.close()
except Exception:
    pass
" 2>/dev/null || true
    echo "🏁 Clean shutdown complete. Motors safely stopped."
}
trap cleanup SIGINT SIGTERM EXIT

cd "$WS_DIR"

# 1. Source ROS 2 base environment and workspace setup
set +u
source /opt/ros/jazzy/setup.bash 2>/dev/null || source /opt/ros/humble/setup.bash 2>/dev/null || true
source /home/etw3/.etw3_camera_env 2>/dev/null || true
if [ -f "$WS_DIR/install/setup.bash" ]; then
    source "$WS_DIR/install/setup.bash" 2>/dev/null || true
else
    echo "⚠️  install/setup.bash not found. Running ./build.sh first..."
    ./build.sh
    source "$WS_DIR/install/setup.bash" 2>/dev/null || true
fi

# 2. Kill any conflicting standard nodes to prevent dual-motor command conflicts
pkill -9 -f "lane_follower" 2>/dev/null || true
pkill -9 -f "lane_offset_publisher" 2>/dev/null || true
pkill -9 -f "web_teleop_node" 2>/dev/null || true
sleep 0.5

# 3. Start camera_node if not already active
if ! pgrep -f camera_node > /dev/null 2>&1; then
    echo "📷 Starting camera_node (640x480)..."
    ros2 run camera_ros camera_node --ros-args -p width:=640 -p height:=480 &
    sleep 2.5
else
    echo "📷 camera_node is already running."
fi

# 4. Start ultrasonic distance publisher if not already active
if ! pgrep -f distance_publisher > /dev/null 2>&1; then
    echo "📏 Starting ultrasonic distance_publisher..."
    ros2 run sensor_nodes distance_publisher &
    sleep 1.0
else
    echo "📏 distance_publisher is already running."
fi

# 5. Start experimental dual-horizon lane detector
echo "👁️  Starting experimental dual-horizon lane detector..."
ros2 run vision_nodes experimental_lane_detector &
sleep 1.0

# 6. Start experimental adaptive lane follower
echo "🚗 Starting experimental adaptive lane follower..."
ros2 run safety_nodes experimental_lane_follower &

echo "========================================================"
echo "✅ All autonomous nodes active! Press Ctrl+C to stop."
echo "========================================================"

# Wait for background processes to keep script running
wait
