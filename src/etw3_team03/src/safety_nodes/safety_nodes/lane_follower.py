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
# Still not turning hard enough at KP=3.0 even with a corrected
# midpoint offset signal - raised further alongside the BASE_DUTY/
# clamp changes below.
KP = 4.0

# Positive lane_offset means the lane is detected to the right of frame
# center, which means the car needs to steer right to re-center on it.
STEER_SIGN = -1

# Stop if we haven't received a lane offset recently
LANE_TIMEOUT_S = 1.0

# Exponential smoothing on lane_offset (0 < x <= 1, 1 = no smoothing).
# Raised from 0.6 - the two-tape midpoint signal is cleaner than the
# old single-contour one, so less filtering is needed and the extra
# lag isn't worth it anymore.
OFFSET_SMOOTHING = 0.8



class LaneFollower(Node):

    def __init__(self):
        super().__init__('lane_follower')

        self.car = Ordinary_Car()

        self.last_distance_time = None
        self.last_distance = None

        self.last_offset_time = None
        self.last_offset = 0.0

        self.offset_msg_count = 0

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
        adjustment = STEER_SIGN * KP * self.last_offset * BASE_DUTY

        # Clamp raised relative to the new lower BASE_DUTY (600) so the
        # inner wheel can drop close to a stall on a hard turn instead
        # of always staying well above zero - sharper turning without
        # reversing either side.
        adjustment = max(-500, min(500, adjustment))

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