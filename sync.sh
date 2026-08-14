#!/usr/bin/env bash
set -e

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

echo "=== 🔄 ETW3 Team Sync Script ==="
echo "Working directory: $REPO_DIR"

cd "$REPO_DIR"

BRANCH=$(git rev-parse --abbrev-ref HEAD 2>/dev/null || echo "main")
echo "1. Fetching latest updates on branch '$BRANCH'..."
git fetch origin "$BRANCH" 2>/dev/null || git fetch origin main

LOCAL=$(git rev-parse HEAD)
REMOTE=$(git rev-parse "origin/$BRANCH" 2>/dev/null || git rev-parse origin/main 2>/dev/null || echo "$LOCAL")

if [ "$LOCAL" != "$REMOTE" ]; then
    echo "⬇️ New changes found on GitHub ($BRANCH). Pulling..."
    git pull --rebase origin "$BRANCH"
else
    echo "✅ Already up-to-date with GitHub ($BRANCH)."
fi

echo "2. Rebuilding ROS 2 workspace with colcon..."
set +u
source /opt/ros/jazzy/setup.bash 2>/dev/null || source /opt/ros/humble/setup.bash 2>/dev/null || true

# Force colcon to scan both top-level src and nested etw3_team03/src
colcon build --symlink-install --base-paths src src/etw3_team03/src

echo "3. Sourcing workspace environment..."
source "$REPO_DIR/install/setup.bash" 2>/dev/null || true

echo "=== 🎉 Sync complete! Workspace rebuilt & sourced. ==="
