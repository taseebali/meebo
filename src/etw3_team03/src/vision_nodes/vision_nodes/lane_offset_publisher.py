import cv2
import numpy as np

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Float32


# Tuned HSV values
HSV_LOWER = np.array([0, 0, 0])
HSV_UPPER = np.array([180, 255, 110])

# Optical Sweet-Spot: Rows 120 to 216 on 480p (25% to 45% of frame height)
# - Completely cuts off room horizon clutter (shoes, chair legs, lab clutter above row 120).
# - Completely avoids near-bumper perspective divergence where tapes spread off-screen (below row 220).
# - Guarantees both tape lines fit within 240px-420px width on 640px sensor.
ROI_TOP_FRAC = 120 / 480
ROI_BOTTOM_FRAC = 216 / 480

# Minimum contour area fraction
MIN_CONTOUR_AREA_FRAC = 100 / (160 * 640)

# Maximum contour bounding-box width fraction (discards shadows/glare wider than 25% of screen)
MAX_CONTOUR_WIDTH_FRAC = 0.25

# Sanity check: reject pair if both candidates are deeply on the same side of the frame center
STRADDLE_MARGIN_FRAC = 0.5

# Max consecutive single-tape frames to extrapolate before falling back to no-data
MAX_SINGLE_TAPE_STREAK = 15

# Throttle logging rate
LOG_EVERY_N = 15

# Dilation kernel applied to the mask to bridge motion blur
DILATE_KERNEL = np.ones((5, 5), np.uint8)

# 2-frame debounce threshold for large offset jumps (>0.35)
MAX_OFFSET_JUMP = 0.35


class LaneOffsetPublisher(Node):

    def __init__(self):
        super().__init__('lane_offset_publisher')

        self.publisher_ = self.create_publisher(
            Float32,
            'lane_offset',
            1
        )

        # Depth 1: zero queued frame backlog
        self.subscription = self.create_subscription(
            CompressedImage,
            '/camera/image_raw/compressed',
            self.on_image,
            1
        )

        self.frame_count = 0

        self.last_published_offset = None
        self.pending_offset = None
        self.pending_count = 0

        self.last_half_gap_px = None
        self.single_tape_streak = 0

        self.get_logger().info(
            'Lane offset publisher started | Sweet-Spot ROI (120-216px)'
        )

        self.get_logger().info(
            'Waiting for camera images on /camera/image_raw/compressed'
        )

    def on_image(self, msg):

        # Decode compressed JPEG/PNG image
        frame = cv2.imdecode(
            np.frombuffer(msg.data, np.uint8),
            cv2.IMREAD_COLOR
        )

        if frame is None:
            self.get_logger().warn('Failed to decode camera frame')
            return

        self.frame_count += 1
        height, width = frame.shape[:2]

        top = max(0, min(int(ROI_TOP_FRAC * height), height))
        bottom = max(top, min(int(ROI_BOTTOM_FRAC * height), height))

        if bottom <= top:
            return

        roi = frame[top:bottom, :]
        min_contour_area = MIN_CONTOUR_AREA_FRAC * roi.shape[0] * roi.shape[1]

        # Convert BGR -> HSV
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

        # Detect lane pixels
        mask = cv2.inRange(hsv, HSV_LOWER, HSV_UPPER)
        mask = cv2.dilate(mask, DILATE_KERNEL, iterations=1)

        frame_center_x = roi.shape[1] / 2.0

        contours, _ = cv2.findContours(
            mask,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE
        )

        def contour_center_x(contour):
            m = cv2.moments(contour)
            return (m['m10'] / m['m00']) if m['m00'] > 0 else (roi.shape[1] / 2.0)

        max_contour_width = MAX_CONTOUR_WIDTH_FRAC * roi.shape[1]

        def is_tape_shaped(contour):
            _, _, w, _ = cv2.boundingRect(contour)
            return w <= max_contour_width

        significant = sorted(
            (
                c for c in contours
                if cv2.contourArea(c) >= min_contour_area
                and is_tape_shaped(c)
            ),
            key=cv2.contourArea,
            reverse=True
        )[:2]

        significant.sort(key=contour_center_x)

        left_tape = significant[0] if len(significant) >= 1 else None
        right_tape = significant[1] if len(significant) >= 2 else None

        if left_tape is not None and right_tape is not None:
            straddle_margin = STRADDLE_MARGIN_FRAC * frame_center_x
            if (
                contour_center_x(left_tape) > frame_center_x + straddle_margin
                or contour_center_x(right_tape) < frame_center_x - straddle_margin
            ):
                left_tape = None
                right_tape = None

        if left_tape is not None and right_tape is not None:
            # Both tapes visible - track midpoint and update live gap memory
            lane_center_x = (
                contour_center_x(left_tape)
                + contour_center_x(right_tape)
            ) / 2.0
            self.last_half_gap_px = (
                contour_center_x(right_tape) - contour_center_x(left_tape)
            ) / 2.0
            self.single_tape_streak = 0
        elif (
            (left_tape is not None or right_tape is not None)
            and self.last_half_gap_px is not None
            and self.single_tape_streak < MAX_SINGLE_TAPE_STREAK
        ):
            # Dynamic single-tape extrapolation
            visible_tape = left_tape if left_tape is not None else right_tape
            visible_x = contour_center_x(visible_tape)
            estimated_center_x = (
                visible_x + self.last_half_gap_px if left_tape is not None
                else visible_x - self.last_half_gap_px
            )

            if not (0 <= estimated_center_x <= roi.shape[1]):
                self.single_tape_streak = 0
                return

            lane_center_x = estimated_center_x
            self.single_tape_streak += 1
        else:
            self.single_tape_streak = 0
            self.last_published_offset = None
            return

        # Normalized offset: -1 = far left, 0 = center, +1 = far right
        offset = (lane_center_x - frame_center_x) / frame_center_x

        # Outlier rejection / Jump debounce
        if (
            self.last_published_offset is not None
            and abs(offset - self.last_published_offset) > MAX_OFFSET_JUMP
        ):
            if (
                self.pending_offset is not None
                and abs(offset - self.pending_offset) <= MAX_OFFSET_JUMP
            ):
                self.pending_count += 1
            else:
                self.pending_offset = offset
                self.pending_count = 1

            if self.pending_count < 2:
                return
        else:
            self.pending_offset = None
            self.pending_count = 0

        self.last_published_offset = offset

        # Publish offset
        msg_out = Float32()
        msg_out.data = float(offset)
        self.publisher_.publish(msg_out)

        if self.frame_count % LOG_EVERY_N == 0:
            tapes_seen = (
                'both' if left_tape is not None and right_tape is not None
                else 'left-only' if left_tape is not None
                else 'right-only'
            )
            self.get_logger().info(
                f'frame={self.frame_count} tapes={tapes_seen} '
                f'lane_x={lane_center_x:.1f} center_x={frame_center_x:.1f} '
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
