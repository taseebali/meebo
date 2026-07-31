import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

# TODO: set this based on what you measured in step 4 — comfortably above
# your sensor's noise floor (so it doesn't fire on jitter alone), at a
# distance where a heads-up warning would actually be useful.
WARN_DISTANCE_CM = None


class DistanceWatch(Node):
    def __init__(self):
        super().__init__('distance_watch')
        # TODO: subscribe to the 'distance_cm' topic (std_msgs/msg/Float32),
        # queue size 10, callback = self.on_distance.
        # self.create_subscription(msg_type, topic, callback, qos) is the
        # subscribing equivalent of self.create_publisher(...) above.

    def on_distance(self, msg):
        # TODO: if msg.data is below WARN_DISTANCE_CM, log a warning with
        # self.get_logger().warn(...). Otherwise, log the reading at info
        # level like the publisher does.
        pass


def main(args=None):
    rclpy.init(args=args)
    node = DistanceWatch()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()