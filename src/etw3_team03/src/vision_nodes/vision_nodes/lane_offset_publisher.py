import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Float32

# TODO: paste your tuned values from S4's tune_threshold.py here.
HSV_LOWER = np.array([30, 2, 0])
HSV_UPPER = np.array([90, 22, 110])

ROI_TOP = 360
ROI_BOTTOM = 800


class LaneOffsetPublisher(Node):
    def __init__(self):
        super().__init__('lane_offset_publisher')
        self.publisher_ = self.create_publisher(Float32, 'lane_offset', 10)
        self.create_subscription(
            CompressedImage, '/camera/image_raw/compressed', self.on_image, 10)

    def on_image(self, msg):
        frame = cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR)
        if frame is None:
            return
        roi = frame[ROI_TOP:ROI_BOTTOM, :]
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, HSV_LOWER, HSV_UPPER)

        moments = cv2.moments(mask)
        if moments['m00'] == 0:
            self.get_logger().warn('No lane pixels found in this frame')
            return

        lane_center_x = moments['m10'] / moments['m00']
        frame_center_x = roi.shape[1] / 2.0
        offset = (lane_center_x - frame_center_x) / frame_center_x

        msg_out = Float32()
        msg_out.data = offset
        self.publisher_.publish(msg_out)


def main(args=None):
    rclpy.init(args=args)
    node = LaneOffsetPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
