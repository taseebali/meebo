#!/usr/bin/env bash
set -e

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

echo "=== 🔄 ETW3 Team Sync Script ==="
echo "Working directory: $REPO_DIR"

cd "$REPO_DIR"

echo "1. Fetching latest updates from GitHub..."
git fetch origin main

LOCAL=$(git rev-parse HEAD)
REMOTE=$(git rev-parse origin/main)

if [ "$LOCAL" != "$REMOTE" ]; then
    echo "⬇️ New changes found on GitHub. Pulling..."
    git pull --rebase origin main
else
    echo "✅ Already up-to-date with GitHub."
fi

echo "🛠️ Rebuilding ROS 2 workspace with colcon..."
set +u
source /opt/ros/jazzy/setup.bash 2>/dev/null || source /opt/ros/humble/setup.bash 2>/dev/null || true
colcon build --symlink-install
echo "✅ ROS 2 workspace rebuilt successfully!"

echo "=== 🎉 Sync complete! ==="
