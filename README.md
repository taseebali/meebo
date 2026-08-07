# ETW III — team repo template

This is your team's ROS 2 workspace. Everything you write for the course —
lane following, the e-stop node, the stop-sign detector, launch files —
lives in `src/` here, as normal ROS 2 packages.

## How this connects to your robot

`etw3-setup/setup.sh` (the Pi provisioning script) clones this repo into
`~/etw3_ws/src/etw3_teamNN` on the robot's Pi and builds it alongside
everything else. Every time you `git push` from your laptop, `git pull &&
colcon build` on the Pi picks it up.

You don't need to vendor any hardware driver code yourselves — the motor
and ultrasonic sensor drivers (`freenove_driver`) are already built into
the workspace by `setup.sh`. From any of your own nodes:

```python
from freenove_driver.motor import Ordinary_Car
from freenove_driver.ultrasonic import Ultrasonic
```

## Layout

```
etw3-team-NN/
  src/            # your ROS 2 packages go here, one directory each
  .gitignore      # excludes build/install/log, __pycache__, bags, model weights
```

There's nothing in `src/` yet on purpose — you'll create your first
package here in S2 (the lab sheet walks you through `ros2 pkg create`).

## Working as a team of two on one workspace

- Branch per feature, PR into `main`, at least one teammate reviews before
  merging — same as any real project.
- Don't commit `build/`, `install/`, or `log/` (already gitignored) —
  they're machine-specific and regenerate from `colcon build`.
- Don't commit rosbags or trained model weights to this repo (see
  `.gitignore`) — the Pi's SD card and your Git history will both thank
  you. Share large files via a drive/cloud folder instead.
- Keep commits scoped to one session's work where you can — it makes the
  "one problem you solved, one thing you'd do differently" part of the
  final presentation much easier to put together in Week 3.

## Safety gate

No autonomous driving before Milestone 1 (S3) is signed off — same rule as
everywhere else in this course. Teleop and bench-testing code before then
is fine and expected.
