# Meebo (ETW3 - Team 03) 🚗🤖

Welcome to Team 03's ROS 2 workspace for the ETW III robot project ("Meebo"). This repository contains our ROS 2 nodes for computer vision, distance sensing, safety E-stop, and motor control running on our Raspberry Pi car setup.

---

## 📂 Repository Structure

* **`src/`**: Contains all ROS 2 packages and helper scripts:
  * **`etw3_team03/`**: Our core team package containing:
    * `vision_nodes/`: Computer vision nodes (`frame_saver`, `lane_offset_publisher`, threshold tuning).
    * `safety_nodes/`: Emergency stop (`estop_node`) and lane follower (`lane_follower`).
    * `sensor_nodes/`: Ultrasonic distance publishers (`distance_publisher`) and watchers (`distance_watch`).
    * `Ashin_Experimental/`: Web Teleop Dashboard with live camera stream & WASD / Gamepad controls.
  * **`freenove_driver/`**: Motor, servo, and ultrasonic sensor hardware drivers.
  * **`teleop_bridge/`**: Remote teleop command velocity bridge (`cmd_vel_bridge`).
* **`kill_all.sh`**: **Master 1-click cleanup script** to stop all background ROS 2 nodes, camera locks, web servers, and motor drivers instantly.
* **`quick-commit.sh`**: One-step script to stage, commit, and push updates directly from the Pi.
* **`sync.sh`**: Helper script to pull latest code from GitHub and rebuild the workspace (`colcon build`).

---

## 🚀 Useful Commands on the Pi

### Reset & Kill all background nodes (Camera / Publishers / Web Server)
```bash
./kill_all.sh
```

### Pull latest code & rebuild workspace
```bash
./sync.sh
```

### Commit & Push changes
```bash
./quick-commit.sh "your update message" [ashin|taseeb]
```

---

## 🛠️ Hardware Drivers Quick Start

In any node script, import the hardware drivers directly:
```python
from freenove_driver.motor import Ordinary_Car
from freenove_driver.ultrasonic import Ultrasonic
```

---

## 👥 Team 03
* **Ashin** ([@AshinMc](https://github.com/AshinMc))
* **Taseeb** ([@taseebali](https://github.com/taseebali))
