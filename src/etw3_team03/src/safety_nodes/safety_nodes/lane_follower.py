import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

from freenove_driver.motor import Ordinary_Car

STOP_DISTANCE_CM = 30
WATCHDOG_TIMEOUT_S = 3.0

BASE_DUTY = 900

# Proportional steering gain
#
# Real-world lane_offset readings while on-track run ~0.1-0.15, but
# KP=2.0 turned out too aggressive once combined with a noisy/biased
# offset signal - it was saturating the +/-300 steering clamp almost
# permanently in one direction (a constant hard turn, not proportional
# correction). Backed off partway between the original 0.5 (too weak
# to turn the wheels at all) and 2.0 (saturates on noise); re-tune on
# the actual track once the offset signal itself is confirmed clean
# (see largest-contour change in lane_offset_publisher.py).
KP = 1.0

# Positive lane_offset means the lane is detected to the right of frame
# center, which means the car needs to steer right to re-center on it.
#
# Full derivation with this robot's documented wiring (positive motor
# duty = backward, see set_motor_model call below): to turn right, the
# left wheels must spin forward faster than the right. Working that
# back through left = BASE_DUTY - adjustment, right = BASE_DUTY +
# adjustment, and set_motor_model(-left, -left, -right, -right) means
# adjustment must be NEGATIVE when offset is positive - which requires
# STEER_SIGN = -1. (A prior change flipped this to +1, which is
# anti-corrective - it steers harder away from the lane the further
# off it gets. If the car still drifts one direction with this value,
# the wiring assumption above is the thing to verify on the bench, not
# this sign.)
STEER_SIGN = -1

# Stop if we haven't received a lane offset recently
LANE_TIMEOUT_S = 1.0

# Exponential smoothing on lane_offset (0 < x <= 1, 1 = no smoothing).
# Raw offset readings occasionally spike to the opposite sign for a
# single frame (mask briefly latching onto a stray pixel cluster) - see
# console captures during tuning. Smoothing keeps one bad frame from
# yanking the wheels the wrong way.
OFFSET_SMOOTHING = 0.3


class LaneFollower(Node):

    def __init__(self):
        super().__init__('lane_follower')

        self.car = Ordinary_Car()

        self.last_distance_time = None
        self.last_distance = None

        self.last_offset_time = None
        self.last_offset = 0.0

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

        self.create_timer(0.1, self.control_loop)

        self.stop_motors()

        self.get_logger().info('Lane follower started')

    def on_distance(self, msg):
        self.last_distance_time = time.monotonic()
        self.last_distance = msg.data

    def on_offset(self, msg):
        self.last_offset_time = time.monotonic()
        self.last_offset = (
            OFFSET_SMOOTHING * msg.data
            + (1 - OFFSET_SMOOTHING) * self.last_offset
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
        adjustment = STEER_SIGN * KP * self.last_offset * BASE_DUTY

        adjustment = max(-300, min(300, adjustment))

        left = BASE_DUTY - adjustment
        right = BASE_DUTY + adjustment

        left = int(left)
        right = int(right)

        self.car.set_motor_model(
            -left,
            -left,
            -right,
            -right
        )

    def stop_motors(self):
        self.car.set_motor_model(0, 0, 0, 0)

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