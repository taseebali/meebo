# Meebo (ETW3 - Team 03)

Welcome to Team 03's ROS 2 workspace for the ETW III robot project ("Meebo"). This repository contains our ROS 2 nodes for computer vision, distance sensing, safety E-stop, and motor control running on our Raspberry Pi car setup.

---

## Repository Structure

* **`src/`**: Contains all ROS 2 packages and helper scripts:
  * **`etw3_team03/`**: Our core team package containing:
    * `vision_nodes/`: Computer vision nodes (frame saving, threshold tuning, lane detection).
    * `safety_nodes/`: Emergency stop (E-stop) and safety monitoring.
    * `sensor_nodes/`: Ultrasonic distance publishers and watchers.
  * **`freenove_driver/`**: Motor, servo, and ultrasonic sensor hardware drivers.
  * **`teleop_bridge/`**: Remote teleop command bridge.
  * **`sample_hsv.py` & `tune_threshold.py`**: Helper utilities for HSV threshold tuning.
* **`quick-commit.sh`**: One-step script to stage, commit, and push updates directly from the Pi.
* **`sync.sh`**: Helper script to sync latest code from GitHub.

---

## Quick Commands on the Pi

### Build the workspace
```bash
cd ~/etw3_ws
colcon build --symlink-install
source install/setup.bash
```

### Commit & Push changes
```bash
./quick-commit.sh "your update message"
```

---

## Hardware Drivers Quick Start

In any node script, import the hardware drivers directly:
```python
from freenove_driver.motor import Ordinary_Car
from freenove_driver.ultrasonic import Ultrasonic
```

---

## Team 03
* **Ashin** ([@AshinMc](https://github.com/AshinMc))
* **Taseeb** ([@taseebali](https://github.com/taseebali))
