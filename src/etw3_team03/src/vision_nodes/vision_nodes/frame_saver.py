import os

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage

SAVE_DIR = os.path.expanduser('~/lane_frames')
SAVE_EVERY_N = 15   # roughly every few seconds — adjust if you want more/fewer


class FrameSaver(Node):
    def __init__(self):
        super().__init__('frame_saver')
        os.makedirs(SAVE_DIR, exist_ok=True)
        self.count = 0
        self.saved = 0
        self.create_subscription(
            CompressedImage, '/camera/image_raw/compressed', self.on_image, 10)
        self.get_logger().info(f'Saving every {SAVE_EVERY_N}th frame to {SAVE_DIR}')

    def on_image(self, msg):
        self.count += 1
        if self.count % SAVE_EVERY_N != 0:
            return
        frame = cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR)
        if frame is None:
            self.get_logger().warn('Got a frame that failed to decode, skipping')
            return
        path = os.path.join(SAVE_DIR, f'frame_{self.saved:03d}.png')
        cv2.imwrite(path, frame)
        self.saved += 1
        self.get_logger().info(f'Saved {path}  shape={frame.shape}')


def main(args=None):
    rclpy.init(args=args)
    node = FrameSaver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
