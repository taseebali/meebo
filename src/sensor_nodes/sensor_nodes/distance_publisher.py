import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

from freenove_driver.ultrasonic import Ultrasonic


class DistancePublisher(Node):
    def __init__(self):
        super().__init__('distance_publisher')
        # TODO: create a publisher on the 'distance_cm' topic
        # (std_msgs/msg/Float32), queue size 10. Store it as
        # self.publisher_ — self.create_publisher(msg_type, topic, qos)
        #
        # TODO: create a timer that calls self.publish_reading on an
        # interval, in seconds (not Hz) — self.create_timer(seconds, cb).
        # Pick the interval based on what you measured in step 4: no
        # point publishing faster than the sensor can actually produce a
        # new reading.

    def publish_reading(self):
        with Ultrasonic() as sensor:
            distance_cm = sensor.get_distance()
        if distance_cm is None:
            self.get_logger().warn('No reading from ultrasonic sensor')
            return
        # TODO: build a Float32 message, set its .data to distance_cm,
        # and publish it on self.publisher_. Log it too
        # (self.get_logger().info(...)) so you can see it in the terminal.


def main(args=None):
    rclpy.init(args=args)
    node = DistancePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
