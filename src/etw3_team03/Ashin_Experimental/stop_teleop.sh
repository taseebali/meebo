#!/usr/bin/env bash

echo "=== 🛑 Stopping Meebo Web Teleop Server Gracefully ==="

# 1. Safely zero motor speeds first via Python
python3 -c "
try:
    from freenove_driver.motor import Ordinary_Car
    car = Ordinary_Car()
    car.set_motor_model(0, 0, 0, 0)
    car.close()
    print('✅ Motors stopped.')
except Exception as e:
    print('Note:', e)
" 2>/dev/null || true

# 2. Send SIGINT (Ctrl+C equivalent) to web_teleop_node so Python executes its cleanup handlers
pkill -2 -f web_teleop_node.py 2>/dev/null || true
pkill -2 -f start_teleop.sh 2>/dev/null || true

sleep 1

# 3. Verify clean shutdown
if pgrep -f web_teleop_node.py > /dev/null; then
    echo "Force stopping remaining processes..."
    pkill -9 -f web_teleop_node.py 2>/dev/null || true
fi

echo "✅ Server stopped cleanly. Motors & camera released."
