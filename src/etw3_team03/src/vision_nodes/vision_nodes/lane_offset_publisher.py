import cv2
import numpy as np

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Float32


# Tuned HSV values
HSV_LOWER = np.array([0, 0, 0])
HSV_UPPER = np.array([180, 255, 110])

# ROI and area threshold are expressed as FRACTIONS of the actual
# incoming frame size, computed at runtime (see on_image below) -
# NOT fixed pixel values. Previous versions hardcoded pixel values
# tuned for one specific resolution (e.g. 640x480), but camera_node's
# resolution depends on a command-line arg that's easy to forget to
# pass - when it silently fell back to its default 800x600, every
# pixel-based ROI/area constant here pointed at the wrong part of the
# image, which is what caused lane tracking to fail/drift on recent
# test runs (confirmed via frame_saver logging shape=(600, 800, 3)
# instead of the assumed 480x640). Fractions stay correct regardless
# of what resolution the camera actually comes up at.
#
# Row closer to the top of the image = ground further ahead of the
# robot (forward-facing, downward-angled camera) - looking too close
# to the robot means a curve is physically underneath it before the
# offset reflects it. But too close to the top = near the vanishing
# point, where the two tape lines visually merge into one blob and
# can't be told apart as separate contours (confirmed via saved
# frames from a sharp-turn test). ROI_TOP_FRAC/ROI_BOTTOM_FRAC below
# picks the same relative band that worked before (was 160-300 out of
# a 480-tall frame).
ROI_TOP_FRAC = 160 / 480
ROI_BOTTOM_FRAC = 300 / 480

# Minimum contour area to trust as "this is a lane line," as a
# fraction of the ROI band's total pixel area (width * height) rather
# than a fixed pixel count - so it scales automatically with whatever
# resolution/ROI size is actually in effect. Derived from the last
# working absolute value (100px on a 140-row x 640-col band).
MIN_CONTOUR_AREA_FRAC = 100 / (140 * 640)

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

        # ROI computed from the ACTUAL frame size every time, not a
        # fixed pixel value - stays correct no matter what resolution
        # camera_node actually comes up at this run.
        top = max(0, min(int(ROI_TOP_FRAC * height), height))
        bottom = max(top, min(int(ROI_BOTTOM_FRAC * height), height))

        if bottom <= top:
            self.get_logger().error(
                f'Invalid ROI: top={top}, bottom={bottom}, '
                f'image_height={height}'
            )
            return

        roi = frame[top:bottom, :]
        min_contour_area = MIN_CONTOUR_AREA_FRAC * roi.shape[0] * roi.shape[1]

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
        # of the lane).
        #
        # Classification used to split contours by a FIXED frame
        # center ("left of center" / "right of center"). That breaks
        # down on a sharp turn: if the robot's heading swings enough,
        # BOTH tapes can end up on the same side of that fixed center,
        # and the classification flips unpredictably frame to frame -
        # this was confirmed on-track (raw_offset whipsawing between
        # -0.4 and +0.24 within ~1.5s during a sharp turn, which a real
        # lane offset never does). Fixed by picking the two largest
        # significant contours and labeling them "left"/"right" purely
        # by their position RELATIVE TO EACH OTHER, not relative to a
        # fixed center - this holds up even when the whole lane has
        # shifted across the frame.
        contours, _ = cv2.findContours(
            mask,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE
        )

        def contour_center_x(contour):
            m = cv2.moments(contour)
            return m['m10'] / m['m00']

        significant = sorted(
            (c for c in contours if cv2.contourArea(c) >= min_contour_area),
            key=cv2.contourArea,
            reverse=True
        )[:2]

        significant.sort(key=contour_center_x)

        left_tape = significant[0] if len(significant) >= 1 else None
        right_tape = significant[1] if len(significant) >= 2 else None

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
