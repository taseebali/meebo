import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

# check_noise.py showed a noise floor of ~0.9-1.8cm spread at fixed
# distances (both empty and with an object in range). 20cm is roughly
# 12x that worst-case spread, so jitter alone can't trigger a false
# warning, while still being far enough out to give a useful heads-up
# rather than firing only once something is nearly touching the sensor.
WARN_DISTANCE_CM = 20.0


class DistanceWatch(Node):
    def __init__(self):
        super().__init__('distance_watch')
        self.subscription = self.create_subscription(
            Float32, 'distance_cm', self.on_distance, 10)

    def on_distance(self, msg):
        if msg.data < WARN_DISTANCE_CM:
            self.get_logger().warn(
                f'Object close: {msg.data:.1f} cm (threshold {WARN_DISTANCE_CM} cm)')
        else:
            self.get_logger().info(f'Distance: {msg.data:.1f} cm')


def main(args=None):
    rclpy.init(args=args)
    node = DistanceWatch()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()