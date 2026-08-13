#!/usr/bin/env python3
"""
Experimental Dual-Horizon Fast Lane Detector with Motion De-blur & Camera Trim.
Features & Mitigations:
1. Camera Alignment Trim (CAMERA_CENTER_TRIM): Calibrates for physically crooked/offset camera mounting.
2. Horizon Roll Leveling (CAMERA_ROLL_ANGLE): Corrects physical roll tilt.
3. Motion De-Blur Filter (Morphological Closing): Reconnects tape fragments broken by motion blur during turns.
4. Single-pass ROI HSV conversion & zero-copy dual-horizon slicing (near centering + far lookahead).
5. Relative contour sorting (fixes left-turn misclassification) & Single-tape fallback.
6. Track Presence Broadcasting (lane_detected: True/False).
"""

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Float32, Bool

# ================= 1. CAMERA MOUNT CALIBRATION =================
# If the camera is physically mounted slightly off-center or angled:
# - Place robot in exact center of straight track.
# - If log reads e.g. near=+0.08, set CAMERA_CENTER_TRIM = 0.08 so calibrated output reads 0.00.
CAMERA_CENTER_TRIM = 0.0

# If the camera is physically tilted/rolled sideways (degrees):
# Positive = counter-clockwise, Negative = clockwise
CAMERA_ROLL_ANGLE = 0.0

# ================= 2. HSV & VISION TUNING =================
HSV_LOWER = np.array([0, 0, 0])
HSV_UPPER = np.array([180, 255, 110])

# Combined ROI bounds (fraction of frame height)
COMBINED_ROI_TOP_FRAC = 100 / 480
COMBINED_ROI_BOTTOM_FRAC = 350 / 480

FAR_SPLIT_RATIO = 0.45
NEAR_SPLIT_RATIO = 0.50

LANE_HALF_WIDTH_PX = 140
MIN_CONTOUR_AREA_FRAC = 100 / (160 * 640)

# De-blur & Morphological kernels
# Wide rectangular kernel bridges horizontal motion blur smears and reconnects fragmented tape
BLUR_BRIDGE_KERNEL = cv2.getStructuringElement(cv2.MORPH_RECT, (7, 3))
DILATE_KERNEL = np.ones((3, 3), np.uint8)

LOG_EVERY_N = 20


class ExperimentalLaneDetector(Node):

    def __init__(self):
        super().__init__('experimental_lane_detector')

        # Publishers
        self.offset_pub = self.create_publisher(Float32, 'lane_offset', 10)
        self.curvature_pub = self.create_publisher(Float32, 'lane_curvature', 10)
        self.detected_pub = self.create_publisher(Bool, 'lane_detected', 10)

        self.subscription = self.create_subscription(
            CompressedImage,
            '/camera/image_raw/compressed',
            self.on_image,
            10
        )

        self.frame_count = 0
        self.get_logger().info(
            f'🚀 Experimental Lane Detector Started | Trim={CAMERA_CENTER_TRIM:+.2f} | Roll={CAMERA_ROLL_ANGLE:.1f}°'
        )

    def process_band(self, band_mask, width, min_area, is_far=False):
        contours, _ = cv2.findContours(band_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return None, 'none'

        def cx(c):
            m = cv2.moments(c)
            return (m['m10'] / m['m00']) if m['m00'] > 0 else (width / 2.0)

        significant = [c for c in contours if cv2.contourArea(c) >= min_area]
        if not significant:
            return None, 'none'

        significant.sort(key=cv2.contourArea, reverse=True)
        top_candidates = significant[:2]
        top_candidates.sort(key=cx)

        left_tape = top_candidates[0]
        right_tape = top_candidates[1] if len(top_candidates) > 1 else None

        frame_cx = width / 2.0
        half_width = LANE_HALF_WIDTH_PX * 0.75 if is_far else LANE_HALF_WIDTH_PX

        if left_tape is not None and right_tape is not None:
            lane_cx = (cx(left_tape) + cx(right_tape)) / 2.0
            status = 'both'
        elif right_tape is not None or left_tape is not None:
            single = right_tape if right_tape is not None else left_tape
            single_cx = cx(single)
            if single_cx >= frame_cx:
                lane_cx = single_cx - half_width
                status = 'right-only'
            else:
                lane_cx = single_cx + half_width
                status = 'left-only'
        else:
            return None, 'none'

        # Calculate raw offset and apply camera mounting trim
        raw_offset = (lane_cx - frame_cx) / frame_cx
        calibrated_offset = raw_offset - CAMERA_CENTER_TRIM
        calibrated_offset = max(-1.0, min(1.0, calibrated_offset))

        return float(calibrated_offset), status

    def on_image(self, msg):
        frame = cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR)
        if frame is None:
            return

        self.frame_count += 1
        height, width = frame.shape[:2]

        # 1. Optional Horizon Leveling (if physical camera is tilted)
        if abs(CAMERA_ROLL_ANGLE) > 0.1:
            rot_mat = cv2.getRotationMatrix2D((width / 2.0, height / 2.0), CAMERA_ROLL_ANGLE, 1.0)
            frame = cv2.warpAffine(frame, rot_mat, (width, height), flags=cv2.INTER_LINEAR)

        # 2. Crop single combined ROI before color conversion
        roi_top = max(0, min(int(COMBINED_ROI_TOP_FRAC * height), height))
        roi_bottom = max(roi_top, min(int(COMBINED_ROI_BOTTOM_FRAC * height), height))

        if roi_bottom <= roi_top:
            return

        roi = frame[roi_top:roi_bottom, :]
        roi_h = roi.shape[0]

        # 3. Fast HSV Thresholding
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, HSV_LOWER, HSV_UPPER)

        # 4. Motion De-blur & Gap Bridging Filter
        # Morphological close connects fragmented blurred lines
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, BLUR_BRIDGE_KERNEL)
        mask = cv2.dilate(mask, DILATE_KERNEL, iterations=1)

        # 5. Dual-Horizon Sub-band Slicing
        far_split = int(FAR_SPLIT_RATIO * roi_h)
        near_split = int(NEAR_SPLIT_RATIO * roi_h)

        far_mask = mask[0:far_split, :]
        near_mask = mask[near_split:roi_h, :]

        min_area_near = MIN_CONTOUR_AREA_FRAC * (near_mask.shape[0] * width)
        min_area_far = min_area_near * 0.6

        near_offset, near_status = self.process_band(near_mask, width, min_area_near, is_far=False)
        far_offset, far_status = self.process_band(far_mask, width, min_area_far, is_far=True)

        # 6. Presence & Offset Publishing
        if near_offset is None and far_offset is None:
            self.detected_pub.publish(Bool(data=False))
            return

        self.detected_pub.publish(Bool(data=True))

        active_offset = near_offset if near_offset is not None else far_offset

        if near_offset is not None and far_offset is not None:
            curvature = far_offset - near_offset
        else:
            curvature = active_offset

        self.offset_pub.publish(Float32(data=float(active_offset)))
        self.curvature_pub.publish(Float32(data=float(curvature)))

        if self.frame_count % LOG_EVERY_N == 0:
            self.get_logger().info(
                f'frame={self.frame_count} | near={active_offset:+.2f} ({near_status}) | '
                f'curv={curvature:+.2f} ({far_status})'
            )


def main(args=None):
    rclpy.init(args=args)
    node = ExperimentalLaneDetector()
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
