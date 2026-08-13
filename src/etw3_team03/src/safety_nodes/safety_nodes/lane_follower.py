import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

from freenove_driver.motor import Ordinary_Car

STOP_DISTANCE_CM = 15
WATCHDOG_TIMEOUT_S = 3.0

BASE_DUTY = 900

# Proportional steering gain
#
# Real-world lane_offset readings while on-track run ~0.1-0.15 (see
# console captures during tuning). At KP=0.5 that was only ~60-70 duty
# out of BASE_DUTY=900 (~7%), too small to overcome drivetrain
# deadzone/stiction - the car never visibly turned. Raised so that
# range of offset produces a differential large enough to actually
# turn the wheels; re-tune on the actual track if it over/under-steers.
KP = 2.0

# Positive lane_offset means the lane is detected to the right of frame
# center, which means the car needs to steer right to re-center on it.
# STEER_SIGN = -1 makes the controller corrective (negative feedback):
# with the BASE_DUTY +/- adjustment convention below and this robot's
# inverted motor wiring (positive duty = backward, see estop_node.py),
# a positive adjustment increases right-wheel forward speed relative to
# left, which turns the car left - so a positive offset needs a
# *negative* adjustment to turn right and correct toward the lane.
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