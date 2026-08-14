#!/usr/bin/env bash
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
if [ -f "$SCRIPT_DIR/../../build.sh" ]; then
    bash "$SCRIPT_DIR/../../build.sh" "$@"
else
    cd /home/etw3/etw3_ws && ./build.sh "$@"
fi
