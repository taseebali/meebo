#!/usr/bin/env bash
set -e

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

if [ -z "$1" ]; then
    echo "Usage: ./quick-commit.sh \"your commit message\" [ashin|taseeb]"
    echo ""
    echo "Examples:"
    echo "  ./quick-commit.sh \"Fixed camera node\""
    echo "  ./quick-commit.sh \"Updated lane detection\" ashin"
    echo "  ./quick-commit.sh \"Fixed motor speed\" taseeb"
    exit 1
fi

MSG="$1"
AUTHOR_ARG="${2,,}"

AUTHOR_FLAG=""
if [ "$AUTHOR_ARG" = "ashin" ] || [ "$AUTHOR_ARG" = "ash" ]; then
    AUTHOR_FLAG='--author=Ashin <ashinalwin3@gmail.com>'
    echo "👤 Commit author: Ashin"
elif [ "$AUTHOR_ARG" = "taseeb" ] || [ "$AUTHOR_ARG" = "tas" ]; then
    AUTHOR_FLAG='--author=Taseeb Ali <alitaseeb@gmail.com>'
    echo "👤 Commit author: Taseeb Ali"
else
    echo "👤 Using default Git author settings"
fi

cd "$REPO_DIR"

echo "=== 🚀 ETW3 Quick Commit & Push ==="
echo "1. Staging changes..."
git add -A

echo "2. Creating commit: \"$MSG\"..."
if [ -n "$AUTHOR_FLAG" ]; then
    git commit $AUTHOR_FLAG -m "$MSG" || echo "No changes to commit."
else
    git commit -m "$MSG" || echo "No changes to commit."
fi

echo "3. Pushing to GitHub (main)..."
git push origin main

echo "✅ Successfully pushed to GitHub!"
