import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

from freenove_driver.ultrasonic import Ultrasonic


class DistancePublisher(Node):
    def __init__(self):
        super().__init__('distance_publisher')
        self.publisher_ = self.create_publisher(Float32, 'distance_cm', 10)

        # check_noise.py measured ~0.71s per reading (21.18s / 30 readings).
        # 0.75s gives a small safety margin above that so we never ask the
        # sensor for a new reading faster than it can actually produce one.
        self.timer = self.create_timer(0.75, self.publish_reading)

    def publish_reading(self):
        with Ultrasonic() as sensor:
            distance_cm = sensor.get_distance()
        if distance_cm is None:
            self.get_logger().warn('No reading from ultrasonic sensor')
            return
        msg = Float32()
        msg.data = distance_cm
        self.publisher_.publish(msg)
        self.get_logger().info(f'Published distance: {distance_cm:.1f} cm')


def main(args=None):
    rclpy.init(args=args)
    node = DistancePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()