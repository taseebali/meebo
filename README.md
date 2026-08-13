# Meebo (ETW3 - Team 03) 🚗🤖

Welcome to Team 03's ROS 2 workspace for the ETW III robot project ("Meebo"). This repository contains our ROS 2 nodes for computer vision, distance sensing, safety E-stop, and motor control running on our Raspberry Pi car setup.

---

## 📂 Repository Structure

* **`src/`**: Contains all ROS 2 packages and helper scripts:
  * **`etw3_team03/`**: Our core team package containing:
    * `vision_nodes/`: Computer vision nodes (`frame_saver`, `lane_offset_publisher`, `experimental_lane_detector`, threshold tuning).
    * `safety_nodes/`: Emergency stop (`estop_node`), lane follower (`lane_follower`), and adaptive follower (`experimental_lane_follower`).
    * `sensor_nodes/`: Ultrasonic distance publishers (`distance_publisher`) and watchers (`distance_watch`).
    * `Ashin_Experimental/`: Web Teleop Dashboard with live camera stream & WASD / Gamepad controls.
  * **`freenove_driver/`**: Motor, servo, and ultrasonic sensor hardware drivers.
  * **`teleop_bridge/`**: Remote teleop command velocity bridge (`cmd_vel_bridge`).
* **`kill_all.sh`**: **Master 1-click cleanup script** to stop all background ROS 2 nodes, camera locks, web servers, and motor drivers instantly.
* **`quick-commit.sh`**: One-step script to stage, commit, and push updates directly from the Pi.
* **`sync.sh`**: Helper script to pull latest code from GitHub and rebuild the workspace (`colcon build`).

---

## 🧪 Experimental: Dual-Horizon Vision, Adaptive Cornering & Smart Track Recovery

In addition to the baseline lane follower, this workspace includes an **Experimental Autonomous Driving Suite** designed to handle sharp bends, improve Raspberry Pi 4 frame rates, and recover automatically if the car drifts off the track.

### 🌟 Key Innovations

#### 1. 👁️ Dual-Horizon Fast Vision (`experimental_lane_detector`)
* **Lookahead Curve Anticipation**: Splits the camera view into two vertical slices:
  * **Near ROI (Rows 220–350)**: Measures current vehicle alignment directly in front of the wheels.
  * **Far Lookahead ROI (Rows 100–210)**: Peeks ahead down the track to measure upcoming lane curvature before the car enters the turn.
* **Left-Turn Horizontal Relative Sorting**: Fixes the bug where sharp turns shifted both tapes into the left half of the image (`< frame_center_x`). Contours are now sorted relative to each other horizontally, ensuring both tapes are always classified correctly.
* **Single-Tape Fallback Estimation**: If the inner line goes out of camera view on a sharp bend, the node uses outer line geometry (`LANE_HALF_WIDTH_PX = 140`) to maintain continuous steering without dropping frames.
* **Raspberry Pi 4 Speed Optimization**: Crops a single sub-region *before* color conversion and uses zero-copy numpy slicing, reducing OpenCV CPU load by **~60%**.
* **Track Presence Broadcasting**: Publishes a real-time `lane_detected` boolean flag (`True`/`False`) so the driver node knows instantly when the car is on or off the track.

#### 2. 🧠 Adaptive Cornering State Machine (`experimental_lane_follower`)
* **Straightaway Cruise Profile**: Runs at `BASE_DUTY = 650` with gentle steering gain (`KP = 1.5`) for fast, stable travel on straight track sections.
* **Cornering Profile**: When curvature or offset $> 0.20$, the state machine switches to `TURNING`, automatically slowing the car down to `BASE_DUTY = 380` and increasing steering gain to `KP = 3.2` with expanded differential wheel torque (`MAX_ADJUST = 380`).
* **Cornering Hysteresis**: Holds the turning profile for at least `0.8s` to prevent steering flutter during multi-apex curves.
* **Slew-Rate Limiter**: Caps maximum steering change per tick (`MAX_SLEW = 85`), eliminating sudden motor snaps and reducing mechanical slip.

#### 3. 🔄 Memory-Guided Track Search & Recovery
If the car loses the track entirely (e.g. overshooting a hairpin or running off-course):
1. **Directional Memory**: The robot remembers whether the track was last seen on the **LEFT** or **RIGHT**.
2. **Phase 1: Memory Pivot (0.0s – 1.2s)**: Cuts forward speed to 0 and executes a slow, controlled in-place pivot toward the last-known side to sweep the camera back across the track.
3. **Phase 2: Expanding Arc Sweep (1.2s – 3.0s)**: If not found in Phase 1, it reverses rotation to sweep across a wider arc on the opposite side.
4. **Phase 3: Re-Acquisition Lock (`RECOVERING`)**: The instant lines reappear, the robot locks into a slow re-centering mode (`BASE_DUTY = 320`, `KP = 3.0`) until centered (`|offset| < 0.12`), then resumes normal high-speed cruising.
5. **Phase 4: Safety Abort Timeout (5.0s)**: Halts motors completely if no track is re-acquired within 5 seconds to prevent runaway behavior.

---

### 🚀 Launching the Experimental Suite on the Pi

#### Option A: Master 1-Click Launch Script
```bash
cd ~/meebo
./sync.sh
./src/etw3_team03/start_experimental.sh
```

#### Option B: Running Individual Nodes via ROS 2
```bash
# Terminal 1: Ultrasonic Distance Sensor
ros2 run sensor_nodes distance_publisher

# Terminal 2: Experimental Fast Dual-Horizon Vision
ros2 run vision_nodes experimental_lane_detector

# Terminal 3: Experimental Adaptive Driver & Recovery
ros2 run safety_nodes experimental_lane_follower
```

---

## ⚙️ Key Parameters & Quick Tuning Reference

| Parameter | Default | Location | Description |
| :--- | :--- | :--- | :--- |
| `STRAIGHT_BASE_DUTY` | `650` | `experimental_lane_follower.py` | Cruise speed on straightaways. |
| `TURN_BASE_DUTY` | `380` | `experimental_lane_follower.py` | Slower speed during curves for grip and sharp turning torque. |
| `STRAIGHT_KP` / `TURN_KP` | `1.5` / `3.2` | `experimental_lane_follower.py` | Proportional steering gains for straight vs. turning modes. |
| `TURN_ENTER_THRESHOLD` | `0.20` | `experimental_lane_follower.py` | Curvature/offset threshold to enter Cornering Mode. |
| `SEARCH_SPIN_DUTY` | `280` | `experimental_lane_follower.py` | In-place pivot speed during track search. |
| `MAX_SEARCH_TIME_S` | `5.0` | `experimental_lane_follower.py` | Safety timeout before aborting track search. |
| `LANE_HALF_WIDTH_PX` | `140` | `experimental_lane_detector.py` | Fallback offset estimation when only 1 tape is visible. |
| `STOP_DISTANCE_CM` | `65` | `experimental_lane_follower.py` | Obstacle E-stop trigger distance. |

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
