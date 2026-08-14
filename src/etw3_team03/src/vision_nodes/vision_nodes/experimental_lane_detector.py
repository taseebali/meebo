#!/usr/bin/env python3
"""
Experimental Dual-Horizon Fast Lane Detector with Motion De-blur, Shadow Filtering,
Dynamic Gap Extrapolation, and Optical Sweet-Spot Geometry (Rows 120-216).

Features & Mitigations:
1. Optical Sweet-Spot ROI (Rows 120-216 / 25%-45% Height):
   - Completely avoids room horizon clutter (shoes, chair legs, lab furniture above row 120).
   - Completely avoids steep near-bumper perspective divergence where tapes exit the screen (below row 220).
2. Dual-Horizon Sub-bands:
   - Far Lookahead Band (Rows 120-163): Measures upcoming curvature 1.5m-2.5m down the track.
   - Near Centering Band (Rows 168-216): Measures immediate vehicle lateral error.
3. Tape Geometry & Shadow Filter (MAX_CONTOUR_WIDTH_FRAC = 0.25): Discards wide floor shadows/glare patches.
4. Center Straddle Sanity Check (STRADDLE_MARGIN_FRAC = 0.5): Prevents same-side false pairings.
5. 2-Frame Jump Debounce (MAX_OFFSET_JUMP = 0.35): Rejects 1-frame transient visual glitches.
6. Dynamic Single-Tape Continuous Gap Memory (MAX_SINGLE_TAPE_STREAK = 15): Bridges sharp corners using last-known gap.
7. Camera Mounting Calibration: CAMERA_CENTER_TRIM and CAMERA_ROLL_ANGLE horizon leveling.
8. Track Presence Broadcasting (lane_detected: True/False).
"""

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Float32, Bool

# ================= 1. CAMERA MOUNT CALIBRATION =================
CAMERA_CENTER_TRIM = 0.0        # Offset for crooked camera mounting
CAMERA_ROLL_ANGLE = 0.0         # Roll leveling in degrees (+ = CCW, - = CW)

# ================= 2. HSV & ROI TUNING =================
HSV_LOWER = np.array([0, 0, 0])
HSV_UPPER = np.array([180, 255, 110])

# Optical Sweet-Spot: Rows 120 to 216 on 480p (25% to 45% of frame height)
# Guaranteed zone where lane width is 240px-420px, safely within 640px sensor width
COMBINED_ROI_TOP_FRAC = 120 / 480       # 0.250 (Row 120)
COMBINED_ROI_BOTTOM_FRAC = 216 / 480    # 0.450 (Row 216)

FAR_SPLIT_RATIO = 0.45          # Rows 120 to 163 for lookahead curvature
NEAR_SPLIT_RATIO = 0.50         # Rows 168 to 216 for near vehicle centering

MIN_CONTOUR_AREA_FRAC = 100 / (160 * 640)
MAX_CONTOUR_WIDTH_FRAC = 0.55   # Accommodate thick corner splices and angled 90-degree turn contours
STRADDLE_MARGIN_FRAC = 0.5      # Sanity check against same-side double picks
MAX_OFFSET_JUMP = 0.35          # 2-frame debounce threshold for large jumps
MAX_SINGLE_TAPE_STREAK = 15     # Max frames to extrapolate single tape on sharp curves

# De-blur & Morphological kernels
BLUR_BRIDGE_KERNEL = cv2.getStructuringElement(cv2.MORPH_RECT, (7, 3))
DILATE_KERNEL = np.ones((3, 3), np.uint8)

LOG_EVERY_N = 20


class ExperimentalLaneDetector(Node):

    def __init__(self):
        super().__init__('experimental_lane_detector')

        # Publishers (QoS Depth 1)
        self.offset_pub = self.create_publisher(Float32, 'lane_offset', 1)
        self.curvature_pub = self.create_publisher(Float32, 'lane_curvature', 1)
        self.detected_pub = self.create_publisher(Bool, 'lane_detected', 1)

        # Subscription with Depth 1 (Zero-Lag Queue)
        self.subscription = self.create_subscription(
            CompressedImage,
            '/camera/image_raw/compressed',
            self.on_image,
            1
        )

        self.frame_count = 0

        # Temporal tracking & debounce state
        self.last_published_offset = None
        self.pending_offset = None
        self.pending_count = 0

        self.last_half_gap_px = None
        self.single_tape_streak = 0

        self.get_logger().info(
            f'🚀 Experimental Lane Detector Started | Sweet-Spot ROI (120-216px) | Trim={CAMERA_CENTER_TRIM:+.2f} | Roll={CAMERA_ROLL_ANGLE:.1f}°'
        )

    def process_band(self, band_mask, width, min_area, is_far=False):
        contours, _ = cv2.findContours(band_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return None, 'none'

        def cx(c):
            m = cv2.moments(c)
            return (m['m10'] / m['m00']) if m['m00'] > 0 else (width / 2.0)

        max_contour_w = MAX_CONTOUR_WIDTH_FRAC * width

        def is_tape_shaped(contour):
            _, _, w, _ = cv2.boundingRect(contour)
            return w <= max_contour_w

        significant = [
            c for c in contours
            if cv2.contourArea(c) >= min_area and is_tape_shaped(c)
        ]
        if not significant:
            return None, 'none'

        significant.sort(key=cv2.contourArea, reverse=True)
        top_candidates = significant[:2]
        top_candidates.sort(key=cx)

        left_tape = top_candidates[0]
        right_tape = top_candidates[1] if len(top_candidates) > 1 else None

        frame_cx = width / 2.0

        # Straddle sanity check: reject if both candidates are deeply on the same side
        if left_tape is not None and right_tape is not None:
            straddle_margin = STRADDLE_MARGIN_FRAC * frame_cx
            if (
                cx(left_tape) > frame_cx + straddle_margin
                or cx(right_tape) < frame_cx - straddle_margin
            ):
                left_tape = None
                right_tape = None

        if left_tape is not None and right_tape is not None:
            lane_cx = (cx(left_tape) + cx(right_tape)) / 2.0
            if not is_far:
                self.last_half_gap_px = (cx(right_tape) - cx(left_tape)) / 2.0
                self.single_tape_streak = 0
            status = 'both'
        elif (
            (left_tape is not None or right_tape is not None)
            and self.last_half_gap_px is not None
            and self.single_tape_streak < MAX_SINGLE_TAPE_STREAK
        ):
            # Single-tape dynamic gap extrapolation
            visible_tape = left_tape if left_tape is not None else right_tape
            visible_x = cx(visible_tape)
            scale = 0.75 if is_far else 1.0
            gap = self.last_half_gap_px * scale

            if left_tape is not None:
                lane_cx = visible_x + gap
                status = 'left-only'
            else:
                lane_cx = visible_x - gap
                status = 'right-only'

            # Boundary validation
            if not (0 <= lane_cx <= width):
                return None, 'none'

            if not is_far:
                self.single_tape_streak += 1
        else:
            if not is_far:
                self.single_tape_streak = 0
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

        # 2. Crop optical sweet-spot ROI (Rows 120 to 216 on 480p)
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

        # 6. Presence Validation
        if near_offset is None and far_offset is None:
            self.detected_pub.publish(Bool(data=False))
            self.last_published_offset = None
            return

        active_offset = near_offset if near_offset is not None else far_offset

        # 7. Two-Frame Jump Debounce for Large Outliers
        if (
            self.last_published_offset is not None
            and abs(active_offset - self.last_published_offset) > MAX_OFFSET_JUMP
        ):
            if (
                self.pending_offset is not None
                and abs(active_offset - self.pending_offset) <= MAX_OFFSET_JUMP
            ):
                self.pending_count += 1
            else:
                self.pending_offset = active_offset
                self.pending_count = 1

            if self.pending_count < 2:
                # Wait for next frame to confirm before publishing large jump
                return
        else:
            self.pending_offset = None
            self.pending_count = 0

        self.last_published_offset = active_offset
        self.detected_pub.publish(Bool(data=True))

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
