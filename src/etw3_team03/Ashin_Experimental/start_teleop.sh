#!/usr/bin/env bash
set -e

echo "=== Launching Meebo Web Teleop Dashboard (Ashin_Experimental) ==="

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$(cd "$SCRIPT_DIR/../../.." && pwd)"

cd "$WS_DIR"

echo "1. Sourcing ROS 2 environment..."
set +u
source /opt/ros/jazzy/setup.bash 2>/dev/null || source /opt/ros/humble/setup.bash 2>/dev/null || true
source "$WS_DIR/install/setup.bash" 2>/dev/null || true

echo "2. Starting Web Teleop Node at http://0.0.0.0:8080..."
python3 "$SCRIPT_DIR/web_teleop_node.py"
