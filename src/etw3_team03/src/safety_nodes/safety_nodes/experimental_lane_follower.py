#!/usr/bin/env python3
"""
Experimental Adaptive Lane Follower with Cornering State Machine.
Features:
1. Fast Cruise Speed on Straightaways (BASE_DUTY = 650).
2. Lookahead Turn Anticipation: Slows down to TURN_BASE_DUTY (380) when a bend approaches.
3. High Turning Authority on Bends: Amplifies KP to 3.2 and expands motor differential.
4. Slew-Rate Limiting: Prevents motor jerk while remaining responsive to real turns.
5. Full Safety Watchdogs: Obstacle detection (65cm) & Sensor / Vision timeouts.
"""

import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from freenove_driver.motor import Ordinary_Car

# Safety settings
STOP_DISTANCE_CM = 65
WATCHDOG_TIMEOUT_S = 3.0
LANE_TIMEOUT_S = 1.5

# ----------------- ADAPTIVE TUNING PROFILES -----------------
# 1. Straight Cruise Profile
STRAIGHT_BASE_DUTY = 650        # Higher cruise speed
STRAIGHT_KP = 1.5               # Gentle steering to prevent wobble
STRAIGHT_MAX_ADJUST = 240       # Clamped steering on straights
STRAIGHT_SLEW_STEP = 45         # Smooth ramp

# 2. Cornering / Curve Profile
TURN_BASE_DUTY = 380            # Slower speed for grip & sharp differential torque
TURN_KP = 3.2                   # Aggressive turning gain
TURN_MAX_ADJUST = 380           # Allows inner wheel to stall/pivot for sharp turns
TURN_SLEW_STEP = 85             # Fast steering transition

# Detection thresholds
TURN_ENTER_THRESHOLD = 0.20     # Offset or curvature magnitude to enter Turn Mode
STRAIGHT_EXIT_THRESHOLD = 0.08  # Offset magnitude to return to Straight Mode
TURN_HOLD_DURATION_S = 0.8      # Minimum duration to stay in Turn Mode once triggered

STEER_SIGN = -1                 # Polarity
OFFSET_SMOOTHING = 0.85         # Exponential smoothing on incoming offset


class ExperimentalLaneFollower(Node):

    def __init__(self):
        super().__init__('experimental_lane_follower')
        self.car = Ordinary_Car()

        self.last_distance_time = None
        self.last_distance = None

        self.last_offset_time = None
        self.last_offset = 0.0
        self.last_curvature = 0.0
        self.last_adjustment = 0.0

        # State machine
        self.mode = 'STRAIGHT'
        self.turn_cooldown_time = 0.0
        self.tick_count = 0

        # ROS 2 Subscriptions
        self.create_subscription(Float32, 'distance_cm', self.on_distance, 10)
        self.create_subscription(Float32, 'lane_offset', self.on_offset, 10)
        self.create_subscription(Float32, 'lane_curvature', self.on_curvature, 10)

        # 20 Hz Control Loop (50ms interval)
        self.create_timer(0.05, self.control_loop)
        self.stop_motors()

        self.get_logger().info('🚀 Experimental Adaptive Lane Follower initialized')

    def on_distance(self, msg):
        self.last_distance_time = time.monotonic()
        self.last_distance = msg.data

    def on_offset(self, msg):
        self.last_offset_time = time.monotonic()
        raw = msg.data
        self.last_offset = (
            OFFSET_SMOOTHING * raw
            + (1.0 - OFFSET_SMOOTHING) * self.last_offset
        )

    def on_curvature(self, msg):
        self.last_curvature = msg.data

    def control_loop(self):
        now = time.monotonic()
        self.tick_count += 1

        # ================= 1. SAFETY CHECKS =================
        # Distance data missing or stale
        if self.last_distance_time is None or (now - self.last_distance_time > WATCHDOG_TIMEOUT_S):
            self.stop_motors()
            return

        # Obstacle detected closer than threshold
        if self.last_distance < STOP_DISTANCE_CM:
            self.stop_motors()
            return

        # Lane data missing or stale
        if self.last_offset_time is None or (now - self.last_offset_time > LANE_TIMEOUT_S):
            self.stop_motors()
            return

        # ================= 2. CORNERING STATE MACHINE =================
        # Check if lane is curving right now or bending ahead in lookahead
        turn_detected = (
            abs(self.last_offset) > TURN_ENTER_THRESHOLD
            or abs(self.last_curvature) > TURN_ENTER_THRESHOLD
        )

        if turn_detected:
            self.mode = 'TURNING'
            self.turn_cooldown_time = now + TURN_HOLD_DURATION_S
        elif now > self.turn_cooldown_time and abs(self.last_offset) < STRAIGHT_EXIT_THRESHOLD:
            self.mode = 'STRAIGHT'

        # Select dynamic parameters
        if self.mode == 'TURNING':
            base_duty = TURN_BASE_DUTY
            kp = TURN_KP
            max_adjust = TURN_MAX_ADJUST
            slew_step = TURN_SLEW_STEP
        else:
            base_duty = STRAIGHT_BASE_DUTY
            kp = STRAIGHT_KP
            max_adjust = STRAIGHT_MAX_ADJUST
            slew_step = STRAIGHT_SLEW_STEP

        # ================= 3. STEERING CONTROL & SLEW =================
        target_adjustment = STEER_SIGN * kp * self.last_offset * base_duty
        target_adjustment = max(-max_adjust, min(max_adjust, target_adjustment))

        # Slew rate limiter: smoothly ramps motor duty towards target
        step = max(-slew_step, min(slew_step, target_adjustment - self.last_adjustment))
        adjustment = self.last_adjustment + step
        self.last_adjustment = adjustment

        left_duty = int(base_duty - adjustment)
        right_duty = int(base_duty + adjustment)

        # Drive motors (negated for robot forward polarity)
        self.car.set_motor_model(
            -left_duty,
            -left_duty,
            -right_duty,
            -right_duty
        )

        if self.tick_count % 15 == 0:
            self.get_logger().info(
                f'[{self.mode}] offset={self.last_offset:+.2f} curv={self.last_curvature:+.2f} | '
                f'speed={base_duty} | L={left_duty} R={right_duty}'
            )

    def stop_motors(self):
        self.car.set_motor_model(0, 0, 0, 0)
        self.last_adjustment = 0.0

    def destroy_node(self):
        self.get_logger().info('Stopping motors and shutting down experimental lane follower')
        try:
            self.stop_motors()
            self.car.close()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ExperimentalLaneFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
