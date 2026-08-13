#!/usr/bin/env python3
"""
Experimental Adaptive Lane Follower with Smart Memory-Guided Track Recovery.
Modes:
1. STRAIGHT: Fast cruise (BASE_DUTY = 650, KP = 1.5).
2. TURNING: Adaptive cornering on bends (BASE_DUTY = 380, KP = 3.2).
3. SEARCHING: When track is lost, uses directional memory to execute a controlled
   in-place pivot toward the last-seen side, followed by an expanding arc sweep.
4. RECOVERING: Soft re-entry lock that pulls the car into lane center before full cruise.
5. SAFETY: Obstacle detection (65cm) & search abort timeout (5.0s).
"""

import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Bool
from freenove_driver.motor import Ordinary_Car

# Safety settings
STOP_DISTANCE_CM = 65
WATCHDOG_TIMEOUT_S = 3.0
LANE_TIMEOUT_S = 1.5

# ----------------- ADAPTIVE TUNING PROFILES -----------------
# 1. Straight Cruise Profile
STRAIGHT_BASE_DUTY = 650
STRAIGHT_KP = 1.5
STRAIGHT_MAX_ADJUST = 240
STRAIGHT_SLEW_STEP = 45

# 2. Cornering / Curve Profile
TURN_BASE_DUTY = 380
TURN_KP = 3.2
TURN_MAX_ADJUST = 380
TURN_SLEW_STEP = 85

# 3. Recovery & Search Tuning
SEARCH_SPIN_DUTY = 280          # Gentle in-place pivot speed
MAX_SEARCH_TIME_S = 5.0         # Abort search after 5s to avoid runaway
SWEEP_PHASE_1_S = 1.2           # Duration of primary search in last-known direction
RECOVERY_BASE_DUTY = 320        # Slower speed while re-centering
RECOVERY_KP = 3.0

# Detection thresholds
TURN_ENTER_THRESHOLD = 0.20
STRAIGHT_EXIT_THRESHOLD = 0.08
TURN_HOLD_DURATION_S = 0.8

STEER_SIGN = -1
OFFSET_SMOOTHING = 0.85


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

        # State machine & Memory
        self.mode = 'STRAIGHT'
        self.lane_detected = False
        self.last_seen_side = -1        # -1 = Left, +1 = Right
        self.search_start_time = None
        self.turn_cooldown_time = 0.0
        self.tick_count = 0

        # Subscriptions
        self.create_subscription(Float32, 'distance_cm', self.on_distance, 10)
        self.create_subscription(Float32, 'lane_offset', self.on_offset, 10)
        self.create_subscription(Float32, 'lane_curvature', self.on_curvature, 10)
        self.create_subscription(Bool, 'lane_detected', self.on_lane_detected, 10)

        # 20 Hz Control Loop
        self.create_timer(0.05, self.control_loop)
        self.stop_motors()

        self.get_logger().info('🚀 Experimental Adaptive Lane Follower with Smart Recovery initialized')

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

        # Update directional memory: which side was the track last located on?
        if abs(raw) > 0.05:
            self.last_seen_side = -1 if raw < 0 else 1

    def on_curvature(self, msg):
        self.last_curvature = msg.data

    def on_lane_detected(self, msg):
        self.lane_detected = msg.data
        if msg.data:
            self.last_offset_time = time.monotonic()

    def control_loop(self):
        now = time.monotonic()
        self.tick_count += 1

        # ================= 1. SAFETY CHECKS =================
        if self.last_distance_time is None or (now - self.last_distance_time > WATCHDOG_TIMEOUT_S):
            self.stop_motors()
            return

        if self.last_distance < STOP_DISTANCE_CM:
            self.stop_motors()
            return

        # ================= 2. SMART SEARCH & RECOVERY =================
        # Triggered when vision reports track lost or offset timeout occurs
        is_lane_lost = (not self.lane_detected) or (self.last_offset_time is not None and (now - self.last_offset_time > 0.35))

        if is_lane_lost:
            if self.search_start_time is None:
                self.search_start_time = now
                self.mode = 'SEARCHING'
                side_str = "LEFT" if self.last_seen_side < 0 else "RIGHT"
                self.get_logger().warn(f'⚠️ Track lost! Starting Memory-Guided Recovery toward {side_str}')

            search_elapsed = now - self.search_start_time

            # Safety abort after max search time
            if search_elapsed > MAX_SEARCH_TIME_S:
                self.get_logger().error('🛑 Track search timed out. Motors safely halted.')
                self.stop_motors()
                return

            # Phase 1: In-place pivot toward last seen side (0 - 1.2s)
            # Phase 2: Reverse sweep across opposite side (1.2s - 3.0s)
            if search_elapsed < SWEEP_PHASE_1_S:
                spin_dir = self.last_seen_side
            else:
                spin_dir = -self.last_seen_side

            # Differential in-place spin (negative duty is forward)
            if spin_dir < 0:
                # Pivot Left: left wheels reverse (+), right wheels forward (-)
                self.car.set_motor_model(
                    +SEARCH_SPIN_DUTY, +SEARCH_SPIN_DUTY,
                    -SEARCH_SPIN_DUTY, -SEARCH_SPIN_DUTY
                )
            else:
                # Pivot Right: left wheels forward (-), right wheels reverse (+)
                self.car.set_motor_model(
                    -SEARCH_SPIN_DUTY, -SEARCH_SPIN_DUTY,
                    +SEARCH_SPIN_DUTY, +SEARCH_SPIN_DUTY
                )
            return

        # ================= 3. RE-ACQUISITION LOCK =================
        if self.search_start_time is not None:
            self.search_start_time = None
            self.mode = 'RECOVERING'
            self.get_logger().info('🎯 Track re-acquired! Locking into re-centering mode.')

        if self.mode == 'RECOVERING':
            if abs(self.last_offset) < 0.12:
                self.mode = 'STRAIGHT'
                self.get_logger().info('✅ Successfully re-centered! Resuming cruise.')

        # ================= 4. NORMAL & CORNERING DRIVE =================
        turn_detected = (
            abs(self.last_offset) > TURN_ENTER_THRESHOLD
            or abs(self.last_curvature) > TURN_ENTER_THRESHOLD
        )

        if turn_detected and self.mode != 'RECOVERING':
            self.mode = 'TURNING'
            self.turn_cooldown_time = now + TURN_HOLD_DURATION_S
        elif self.mode == 'TURNING' and now > self.turn_cooldown_time and abs(self.last_offset) < STRAIGHT_EXIT_THRESHOLD:
            self.mode = 'STRAIGHT'

        # Select dynamic parameters
        if self.mode == 'RECOVERING':
            base_duty = RECOVERY_BASE_DUTY
            kp = RECOVERY_KP
            max_adjust = 320
            slew_step = 80
        elif self.mode == 'TURNING':
            base_duty = TURN_BASE_DUTY
            kp = TURN_KP
            max_adjust = TURN_MAX_ADJUST
            slew_step = TURN_SLEW_STEP
        else:
            base_duty = STRAIGHT_BASE_DUTY
            kp = STRAIGHT_KP
            max_adjust = STRAIGHT_MAX_ADJUST
            slew_step = STRAIGHT_SLEW_STEP

        target_adjustment = STEER_SIGN * kp * self.last_offset * base_duty
        target_adjustment = max(-max_adjust, min(max_adjust, target_adjustment))

        step = max(-slew_step, min(slew_step, target_adjustment - self.last_adjustment))
        adjustment = self.last_adjustment + step
        self.last_adjustment = adjustment

        left_duty = int(base_duty - adjustment)
        right_duty = int(base_duty + adjustment)

        self.car.set_motor_model(
            -left_duty,
            -left_duty,
            -right_duty,
            -right_duty
        )

        if self.tick_count % 15 == 0:
            self.get_logger().info(
                f'[{self.mode}] offset={self.last_offset:+.2f} | speed={base_duty} | L={left_duty} R={right_duty}'
            )

    def stop_motors(self):
        self.car.set_motor_model(0, 0, 0, 0)
        self.last_adjustment = 0.0

    def destroy_node(self):
        self.get_logger().info('Shutting down experimental lane follower')
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
