#!/bin/bash
# Helper script to launch experimental dual-horizon lane following on the Pi

echo "========================================================"
echo "🏎️  Launching Experimental Dual-Horizon Lane Follower"
echo "========================================================"

# Trap SIGINT to cleanly kill child processes
trap 'kill $(jobs -p) 2>/dev/null' EXIT

# Start ultrasonic distance publisher if not already running
ros2 run sensor_nodes distance_publisher &
DIST_PID=$!

# Start experimental dual-horizon lane detector
ros2 run vision_nodes experimental_lane_detector &
VISION_PID=$!

# Start experimental adaptive lane follower
ros2 run safety_nodes experimental_lane_follower &
FOLLOWER_PID=$!

echo "✅ All experimental nodes started. Press Ctrl+C to stop."
wait
