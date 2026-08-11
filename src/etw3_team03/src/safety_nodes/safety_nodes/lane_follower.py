import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

from freenove_driver.motor import Ordinary_Car

# From S3. Reuse your signed-off values, don't re-guess them.
STOP_DISTANCE_CM = 50   # TODO
WATCHDOG_TIMEOUT_S = 3.0  # TODO

BASE_DUTY = 1200

# TODO: proportional gain. Start at 0, see step 1's tuning procedure.
KP = 1.5

# TODO: +1 if reducing the LEFT side's duty turned the robot left; -1 if
# it turned the robot right (from step 2).
STEER_SIGN = 1


class LaneFollower(Node):
    def __init__(self):
        super().__init__('lane_follower')
        self.car = Ordinary_Car()
        self.last_distance_time = None
        self.last_distance = None
        self.last_offset = 0.0

        self.create_subscription(Float32, 'distance_cm', self.on_distance, 10)
        self.create_subscription(Float32, 'lane_offset', self.on_offset, 10)
        self.create_timer(0.1, self.control_loop)

        self.stop_motors()

    def on_distance(self, msg):
        self.last_distance_time = time.monotonic()
        self.last_distance = msg.data

    def on_offset(self, msg):
        self.last_offset = msg.data

    def control_loop(self):
        # Safety first, every cycle. Same shape as S3's estop_node. This
        # is the only node commanding motors right now; real multi-node
        # arbitration is Thursday's integration work.
        if self.last_distance_time is None:
            self.stop_motors()
            return
        if time.monotonic() - self.last_distance_time > WATCHDOG_TIMEOUT_S:
            self.stop_motors()
            return
        if self.last_distance < STOP_DISTANCE_CM:
            self.stop_motors()
            return

        # TODO: compute a steering adjustment from self.last_offset, KP,
        # and STEER_SIGN, e.g.:
        #   adjustment = STEER_SIGN * KP * self.last_offset * BASE_DUTY
        # then bias one side up and the other down before calling
        # self.car.set_motor_model(fl, bl, fr, br). Clamp to a sane range
        # (e.g. 0-4095). An unclamped adjustment can produce nonsense
        # duty at a large offset.

    def stop_motors(self):
        self.car.set_motor_model(0, 0, 0, 0)

    def destroy_node(self):
        self.stop_motors()
        self.car.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = LaneFollower()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
