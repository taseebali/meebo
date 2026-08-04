import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

from freenove_driver.motor import Ordinary_Car

# 57 cm/s measured speed x 0.75s S2 publish interval = 42.75cm raw reaction
# distance, doubled for margin (starting point - test and adjust on the
# actual demo floor).
STOP_DISTANCE_CM = 85

# 0.75s publish interval x 4 - a missed reading or two shouldn't trip this,
# but the sensor going properly silent should catch it fast.
WATCHDOG_TIMEOUT_S = 3.0

DRIVE_DUTY = 1200   # matches the duty used for the 57cm/s speed measurement


class EstopNode(Node):
    def __init__(self):
        super().__init__('estop_node')
        self.car = Ordinary_Car()
        self.last_reading_time = None

        self.create_subscription(Float32, 'distance_cm', self.on_distance, 10)
        # Runs independently of on_distance - this is what catches a
        # sensor that's gone silent, not just a sensor reporting "close."
        self.create_timer(0.1, self.check_watchdog)

        # Safe default at startup: don't drive until we've heard from the
        # sensor at least once.
        self.stop_motors()

    def on_distance(self, msg):
        self.last_reading_time = time.monotonic()
        if msg.data < STOP_DISTANCE_CM:
            self.stop_motors()
        else:
            self.drive_forward()

    def check_watchdog(self):
        if self.last_reading_time is None:
            return   # on_distance handles the "never heard anything yet" case
        if time.monotonic() - self.last_reading_time > WATCHDOG_TIMEOUT_S:
            self.stop_motors()

    def drive_forward(self):
        # Negated: this robot's wiring means positive duty drives backward.
        # Fix at the hardware level (swap motor leads) and this negation
        # can go away.
        self.car.set_motor_model(-DRIVE_DUTY, -DRIVE_DUTY, -DRIVE_DUTY, -DRIVE_DUTY)

    def stop_motors(self):
        self.car.set_motor_model(0, 0, 0, 0)

    def destroy_node(self):
        self.stop_motors()
        self.car.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = EstopNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()