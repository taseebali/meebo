#!/usr/bin/env bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$(cd "$SCRIPT_DIR/../../.." && pwd)"

echo "========================================================"
echo "🏎️  Launching Experimental Dual-Horizon Lane Follower"
echo "========================================================"

# Trap SIGINT to cleanly kill child processes
trap 'kill $(jobs -p) 2>/dev/null' EXIT

cd "$WS_DIR"
set +u
source /opt/ros/jazzy/setup.bash 2>/dev/null || source /opt/ros/humble/setup.bash 2>/dev/null || true
source /home/etw3/.etw3_camera_env 2>/dev/null || true
source "$WS_DIR/install/setup.bash" 2>/dev/null || true

# 1. Start ultrasonic distance publisher
ros2 run sensor_nodes distance_publisher &
DIST_PID=$!

# 2. Start experimental dual-horizon lane detector
ros2 run vision_nodes experimental_lane_detector &
VISION_PID=$!

# 3. Start experimental adaptive lane follower
ros2 run safety_nodes experimental_lane_follower &
FOLLOWER_PID=$!

echo "✅ All experimental nodes started. Press Ctrl+C to stop."
wait
