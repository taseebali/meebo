import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from freenove_driver.motor import Ordinary_Car

LINEAR_SCALE = 2000.0    # duty units per 1.0 m/s of linear.x
ANGULAR_SCALE = 2000.0   # duty units per 1.0 rad/s of angular.z
WATCHDOG_TIMEOUT = 0.5   # seconds - stop if cmd_vel goes quiet (dropped keyboard/network)


class CmdVelBridge(Node):
    def __init__(self):
        super().__init__('cmd_vel_bridge')
        self.car = Ordinary_Car()
        self.last_msg_time = self.get_clock().now()
        self.create_subscription(Twist, 'cmd_vel', self.on_cmd_vel, 10)
        self.create_timer(0.1, self.watchdog_check)

    def on_cmd_vel(self, msg: Twist):
        self.last_msg_time = self.get_clock().now()
        left = LINEAR_SCALE * msg.linear.x - ANGULAR_SCALE * msg.angular.z
        right = LINEAR_SCALE * msg.linear.x + ANGULAR_SCALE * msg.angular.z
        left = max(-4095, min(4095, int(left)))
        right = max(-4095, min(4095, int(right)))
        self.car.set_motor_model(left, left, right, right)

    def watchdog_check(self):
        elapsed = (self.get_clock().now() - self.last_msg_time).nanoseconds / 1e9
        if elapsed > WATCHDOG_TIMEOUT:
            self.car.set_motor_model(0, 0, 0, 0)

    def destroy_node(self):
        self.car.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
