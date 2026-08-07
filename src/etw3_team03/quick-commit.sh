#!/usr/bin/env bash
set -e

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

if [ -z "$1" ]; then
    echo "Usage: ./quick-commit.sh \"your commit message\""
    exit 1
fi

MSG="$1"

cd "$REPO_DIR"

echo "=== 🚀 ETW3 Quick Commit & Push ==="
echo "1. Staging changes..."
git add -A

echo "2. Creating commit: \"$MSG\"..."
git commit -m "$MSG" || echo "No changes to commit."

echo "3. Pushing to GitHub (main)..."
git push origin main

echo "✅ Successfully pushed to GitHub!"

