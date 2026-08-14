#!/usr/bin/env bash
set -e

echo "========================================================"
echo "🛠️  Building ROS 2 Workspace for Autonomous Robot"
echo "========================================================"

# Determine workspace root
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
if [ -f "$SCRIPT_DIR/install/setup.bash" ] || [ -d "$SCRIPT_DIR/src" ]; then
    WS_DIR="$SCRIPT_DIR"
elif [ -d "/home/etw3/etw3_ws" ]; then
    WS_DIR="/home/etw3/etw3_ws"
else
    WS_DIR="$(cd "$SCRIPT_DIR/../.." && pwd)"
fi

cd "$WS_DIR"
echo "📂 Building in workspace: $WS_DIR"

# 1. Source ROS 2 base environment
set +u
source /opt/ros/jazzy/setup.bash 2>/dev/null || source /opt/ros/humble/setup.bash 2>/dev/null || true
source /home/etw3/.etw3_camera_env 2>/dev/null || true

# 2. Build all packages with symlink-install
echo "⚙️  Running colcon build..."
colcon build --symlink-install --base-paths src src/etw3_team03/src --packages-select freenove_driver sensor_nodes vision_nodes safety_nodes teleop_bridge 2>&1 || colcon build --symlink-install --base-paths src src/etw3_team03/src

# 3. Source the updated workspace installation
source "$WS_DIR/install/setup.bash"

echo "========================================================"
echo "✅ Build complete! All nodes are registered and ready:"
echo "   - experimental_lane_detector"
echo "   - experimental_lane_follower"
echo "   - lane_offset_publisher"
echo "   - lane_follower"
echo "   - distance_publisher"
echo "   - estop_node"
echo "========================================================"
