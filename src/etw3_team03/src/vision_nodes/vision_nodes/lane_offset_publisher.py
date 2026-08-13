import cv2
import numpy as np

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Float32


# Tuned HSV values
HSV_LOWER = np.array([0, 0, 0])
HSV_UPPER = np.array([180, 255, 110])

# Narrower band, moved further up the frame than the previous 300-700.
# A row closer to the top of the image corresponds to ground further
# ahead of the robot (forward-facing, downward-angled camera) - the
# old ROI only looked at ground close to the robot, so a curve was
# physically underneath it before the offset reflected it at all.
# This trades some precision for lookahead: re-tune on the actual
# track (move up further for more lookahead, down for more precision
# on tight curves) once basic reaction timing is confirmed OK.
ROI_TOP = 150
ROI_BOTTOM = 350

# Minimum contour area (in pixels) to trust as "this is a lane line."
# Filters out shadows/noise/small dark specs that would otherwise be
# picked up as a false lane detection.
MIN_CONTOUR_AREA = 200

# Logging every frame (at full camera rate) is expensive on a Pi and was
# eating into the time available for actual frame processing, adding
# latency to the offset the lane follower reacts to. Throttle it.
LOG_EVERY_N = 15


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

        # Calculate center of image
        frame_center_x = roi.shape[1] / 2.0

        # The track is TWO separate tape lines, not one - the robot
        # should track the MIDPOINT between them, not center itself on
        # either tape individually (which just makes it hug one edge
        # of the lane). Find significant contours, split them into
        # "left of frame center" / "right of frame center" groups by
        # centroid, and take the largest contour on each side as that
        # side's tape line.
        contours, _ = cv2.findContours(
            mask,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE
        )

        def contour_center_x(contour):
            m = cv2.moments(contour)
            return m['m10'] / m['m00']

        significant = [
            c for c in contours
            if cv2.contourArea(c) >= MIN_CONTOUR_AREA
        ]

        left_candidates = [
            c for c in significant
            if contour_center_x(c) < frame_center_x
        ]
        right_candidates = [
            c for c in significant
            if contour_center_x(c) >= frame_center_x
        ]

        left_tape = (
            max(left_candidates, key=cv2.contourArea)
            if left_candidates else None
        )
        right_tape = (
            max(right_candidates, key=cv2.contourArea)
            if right_candidates else None
        )

        if left_tape is not None and right_tape is not None:
            # Both tapes visible - track the midpoint between them
            lane_center_x = (
                contour_center_x(left_tape)
                + contour_center_x(right_tape)
            ) / 2.0
        else:
            # Only one (or no) tape visible. Previously this estimated
            # the lane center using a fixed HALF_LANE_WIDTH_PX guess,
            # but that constant was never measured against the real
            # track and camera perspective makes the true tape gap
            # shrink further from the robot - a wrong-but-confident
            # guess here produced a consistent steering bias (see
            # on-track testing). Safer to treat this the same as "no
            # lane pixels" and let LANE_TIMEOUT_S stop the car briefly
            # rather than actively steer it on an unvalidated estimate.
            if self.frame_count % LOG_EVERY_N == 0:
                self.get_logger().warn(
                    'Only one tape visible - treating as no lane data '
                    '(single-tape estimate disabled, see comment above)'
                )
            return

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

        # Console output (throttled - logging every frame at full camera
        # rate was adding noticeable latency on the Pi)
        if self.frame_count % LOG_EVERY_N == 0:
            tapes_seen = (
                'both' if left_tape is not None and right_tape is not None
                else 'left-only' if left_tape is not None
                else 'right-only'
            )
            self.get_logger().info(
                f'frame={self.frame_count} '
                f'tapes={tapes_seen} '
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
