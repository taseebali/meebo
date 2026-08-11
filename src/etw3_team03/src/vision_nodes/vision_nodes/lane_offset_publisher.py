import cv2
import numpy as np

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Float32


# Tuned HSV values
HSV_LOWER = np.array([30, 2, 0])
HSV_UPPER = np.array([90, 22, 110])

ROI_TOP = 360
ROI_BOTTOM = 800


class LaneOffsetPublisher(Node):

    def __init__(self):
        super().__init__('lane_offset_publisher')

        self.publisher_ = self.create_publisher(
            Float32,
            'lane_offset',
            10
        )

        self.subscription = self.create_subscription(
            CompressedImage,
            '/camera/image_raw/compressed',
            self.on_image,
            10
        )

        self.frame_count = 0

        self.get_logger().info(
            'Lane offset publisher started'
        )

        self.get_logger().info(
            'Waiting for camera images on '
            '/camera/image_raw/compressed'
        )

    def on_image(self, msg):

        # Decode compressed JPEG/PNG image
        frame = cv2.imdecode(
            np.frombuffer(msg.data, np.uint8),
            cv2.IMREAD_COLOR
        )

        if frame is None:
            self.get_logger().warn(
                'Failed to decode camera frame'
            )
            return

        self.frame_count += 1

        height, width = frame.shape[:2]

        # Make sure ROI is inside the actual image
        top = max(0, min(ROI_TOP, height))
        bottom = max(top, min(ROI_BOTTOM, height))

        if bottom <= top:
            self.get_logger().error(
                f'Invalid ROI: top={top}, bottom={bottom}, '
                f'image_height={height}'
            )
            return

        roi = frame[top:bottom, :]

        # Convert BGR -> HSV
        hsv = cv2.cvtColor(
            roi,
            cv2.COLOR_BGR2HSV
        )

        # Detect lane pixels
        mask = cv2.inRange(
            hsv,
            HSV_LOWER,
            HSV_UPPER
        )

        # Calculate image moments
        moments = cv2.moments(mask)

        if moments['m00'] == 0:
            self.get_logger().warn(
                'No lane pixels found in this frame'
            )
            return

        # Calculate lane center
        lane_center_x = (
            moments['m10'] / moments['m00']
        )

        # Calculate center of image
        frame_center_x = roi.shape[1] / 2.0

        # Normalized offset:
        #
        #   -1 = far left
        #    0 = center
        #   +1 = far right
        #
        offset = (
            lane_center_x - frame_center_x
        ) / frame_center_x

        # Publish offset
        msg_out = Float32()
        msg_out.data = float(offset)

        self.publisher_.publish(msg_out)

        # Console output
        self.get_logger().info(
            f'frame={self.frame_count} '
            f'lane_x={lane_center_x:.1f} '
            f'center_x={frame_center_x:.1f} '
            f'offset={offset:.3f}'
        )


def main(args=None):
    rclpy.init(args=args)

    node = LaneOffsetPublisher()

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