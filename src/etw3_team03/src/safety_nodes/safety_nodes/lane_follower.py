import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

from freenove_driver.motor import Ordinary_Car

STOP_DISTANCE_CM = 65
WATCHDOG_TIMEOUT_S = 3.0

# Lowered from 900: slower driving gives the vision/control loop more
# time to react per unit distance travelled (shorter physical stopping
# distance too), and makes the steering clamp below a much bigger
# fraction of drive speed, so turns are sharper at the same clamp.
BASE_DUTY = 600

# Proportional steering gain
# KP=4.0 (with the +/-500 clamp below) was confirmed too aggressive
# via video of an actual test run: on a real curve it didn't turn
# proportionally, it slammed into a near-max rotation, overshot past
# facing the right way, then swung back the other way - visible as
# the car spinning ~90 degrees and rocking back rather than smoothly
# following the bend. That's oscillation from too much gain, not a
# wrong-direction bug (the sign was verified correct/consistent
# throughout the same test's logs). Backed off to reduce overshoot.
KP = 2.0

# Positive lane_offset means the lane is detected to the right of frame
# center, which means the car needs to steer right to re-center on it.
STEER_SIGN = -1

# Stop if we haven't received a lane offset recently
# Raised from 1.0: live testing showed a sharp turn causing a real,
# multi-second vision dropout (motion blur + tapes converging near
# the vanishing point - see mask dilation fix in
# lane_offset_publisher.py, which targets the root cause). This gives
# a bit more grace before a full stop for whatever gap remains after
# that fix, without weakening the timeout as a genuine safety net for
# sustained lane loss.
LANE_TIMEOUT_S = 1.5

# Exponential smoothing on lane_offset (0 < x <= 1, 1 = no smoothing).
# Raised from 0.6 - the two-tape midpoint signal is cleaner than the
# old single-contour one, so less filtering is needed and the extra
# lag isn't worth it anymore.
OFFSET_SMOOTHING = 0.8

# Max change in `adjustment` allowed per control_loop tick (runs at
# 20Hz - see timer below). On-track testing showed a single bad/
# ambiguous vision frame could snap the commanded adjustment straight
# from one clamp extreme to the other (+500 to -500) in about a
# second, which threw the robot off the marked path at a sharp turn
# instead of tracking it. This limits how fast the commanded steering
# can move regardless of how much the input signal jumps - real turns
# still get there, just ramped instead of snapped.
MAX_ADJUSTMENT_STEP = 60



class LaneFollower(Node):

    def __init__(self):
        super().__init__('lane_follower')

        self.car = Ordinary_Car()

        self.last_distance_time = None
        self.last_distance = None

        self.last_offset_time = None
        self.last_offset = 0.0

        self.offset_msg_count = 0
        self.last_adjustment = 0.0

        self.create_subscription(
            Float32,
            'distance_cm',
            self.on_distance,
            10
        )

        self.create_subscription(
            Float32,
            'lane_offset',
            self.on_offset,
            10
        )

        # Faster control loop (was 0.1s/10Hz) - reacts to the latest
        # received offset/distance sooner instead of sitting on stale
        # values for up to 100ms between checks.
        self.create_timer(0.05, self.control_loop)

        self.stop_motors()

        self.get_logger().info('Lane follower started')

    def on_distance(self, msg):
        self.last_distance_time = time.monotonic()
        self.last_distance = msg.data

    def on_offset(self, msg):
        self.last_offset_time = time.monotonic()
        raw = msg.data
        self.last_offset = (
            OFFSET_SMOOTHING * raw
            + (1 - OFFSET_SMOOTHING) * self.last_offset
        )
        self.offset_msg_count += 1
        if self.offset_msg_count % 5 == 0:
            self.get_logger().info(
                f'DEBUG: raw_offset={raw:.3f} '
                f'smoothed_offset={self.last_offset:.3f} '
                f'({"lane right of center, should steer RIGHT" if raw > 0 else "lane left of center, should steer LEFT"})'
            )

    def control_loop(self):

        # -----------------------------
        # SAFETY: no distance received
        # -----------------------------
        if self.last_distance_time is None:
            self.stop_motors()
            return

        # -----------------------------
        # SAFETY: distance data stale
        # -----------------------------
        if time.monotonic() - self.last_distance_time > WATCHDOG_TIMEOUT_S:
            self.get_logger().warn(
                'Distance watchdog timeout - stopping motors'
            )
            self.stop_motors()
            return

        # -----------------------------
        # SAFETY: obstacle too close
        # -----------------------------
        if self.last_distance < STOP_DISTANCE_CM:
            self.stop_motors()
            return

        # -----------------------------
        # SAFETY: no lane data
        # -----------------------------
        if self.last_offset_time is None:
            self.stop_motors()
            return

        # -----------------------------
        # SAFETY: lane data stale
        # -----------------------------
        if time.monotonic() - self.last_offset_time > LANE_TIMEOUT_S:
            self.get_logger().warn(
                'Lane offset timeout - stopping motors'
            )
            self.stop_motors()
            return

        # -----------------------------
        # PROPORTIONAL STEERING
        # -----------------------------
        target_adjustment = STEER_SIGN * KP * self.last_offset * BASE_DUTY

        # Lowered from 500 alongside the KP reduction above - at 500 a
        # single sharp reading could still hit near-max rotation even
        # with lower gain. 300 (half of BASE_DUTY=600) still lets the
        # inner wheel slow to a near-stall on a real hard turn, without
        # the same overshoot magnitude that caused the spin-and-rock
        # behavior seen on video.
        target_adjustment = max(-300, min(300, target_adjustment))

        # Rate-limit: move last_adjustment toward target_adjustment by
        # at most MAX_ADJUSTMENT_STEP this tick, instead of jumping
        # straight to it.
        step = max(
            -MAX_ADJUSTMENT_STEP,
            min(MAX_ADJUSTMENT_STEP, target_adjustment - self.last_adjustment)
        )
        adjustment = self.last_adjustment + step
        self.last_adjustment = adjustment

        left = BASE_DUTY - adjustment
        right = BASE_DUTY + adjustment

        left = int(left)
        right = int(right)

        if self.offset_msg_count % 5 == 0:
            self.get_logger().info(
                f'DEBUG: adjustment={adjustment:.1f} '
                f'left_duty={left} right_duty={right} '
                f'({"left wheels faster -> turning RIGHT" if left > right else "right wheels faster -> turning LEFT"})'
            )

        self.car.set_motor_model(
            -left,
            -left,
            -right,
            -right
        )

    def stop_motors(self):
        self.car.set_motor_model(0, 0, 0, 0)
        self.last_adjustment = 0.0

    def destroy_node(self):
        self.get_logger().info('Stopping motors and shutting down')

        try:
            self.stop_motors()
            self.car.close()
        except Exception as e:
            self.get_logger().warn(
                f'Error while closing motor controller: {e}'
            )

        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    node = LaneFollower()

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