#!/usr/bin/env python3
"""
Experimental Dual-Horizon Fast Lane Detector with Temporal Association Tracking,
Shadow Filtering, Dynamic Gap Extrapolation, and Proven Horizon-Safe ROI (Rows 144-240).

Features & Mitigations:
1. Proven Horizon-Safe ROI (Rows 144-240 / 30%-50% Height):
   - Proven on-track: avoids background shoes, chair legs, and chrome stool reflections.
   - Dual-horizon slicing: Far lookahead (rows 144-187) + Near centering (rows 192-240).
2. Temporal Association Tracking (prev_left_x, prev_right_x):
   - Latches onto the real physical tape lines across frames instead of re-deciding by area.
   - Completely eliminates jumping to background dark objects (shoes, furniture).
3. Single-Candidate Memory Association:
   - Identifies which side a single visible tape belongs to using temporal memory.
4. Dynamic Single-Tape Continuous Gap Extrapolation (MAX_SINGLE_TAPE_STREAK = 15):
   - Bridges sharp 90-degree corners using live measured tape gap.
5. Geometry & Shadow Filter (MAX_CONTOUR_WIDTH_FRAC = 0.55):
   - Accommodates thick corner splices while rejecting whole-frame lighting gradients.
6. 2-Frame Jump Debounce (MAX_OFFSET_JUMP = 0.35).
7. Track Presence Broadcasting (lane_detected: True/False).
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

# Proven on-track band: Rows 144 to 240 on 480p (180 to 300 on 600p)
# Cleanly isolates both tapes while cutting off horizon shoes & chrome stool base
COMBINED_ROI_TOP_FRAC = 144 / 480       # 0.300 (Row 144)
COMBINED_ROI_BOTTOM_FRAC = 240 / 480    # 0.500 (Row 240)

FAR_SPLIT_RATIO = 0.45          # Rows 144 to 187 for lookahead curvature
NEAR_SPLIT_RATIO = 0.50         # Rows 192 to 240 for near vehicle centering

MIN_CONTOUR_AREA_FRAC = 100 / (160 * 640)
MAX_CONTOUR_WIDTH_FRAC = 0.55   # Accommodate thick corner splices and angled 90-degree turn contours
STRADDLE_MARGIN_FRAC = 0.5      # Sanity check against same-side double picks
MAX_OFFSET_JUMP = 0.35          # 2-frame debounce threshold for large jumps
MAX_SINGLE_TAPE_STREAK = 35     # Max frames to extrapolate single tape on sharp 90-degree curves (~1.75s)

MAX_ASSOCIATION_SHIFT_FRAC = 0.25
NO_DATA_RESET_STREAK = 15

# De-blur & Morphological kernels
BLUR_BRIDGE_KERNEL = cv2.getStructuringElement(cv2.MORPH_RECT, (7, 3))
DILATE_KERNEL = np.ones((5, 5), np.uint8)

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

        # Temporal association memory
        self.prev_near_left_x = None
        self.prev_near_right_x = None
        self.no_data_streak = 0

        self.get_logger().info(
            f'🚀 Experimental Lane Detector with Temporal Association Started | ROI (144-240px)'
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

        candidates = sorted(
            [
                c for c in contours
                if cv2.contourArea(c) >= min_area and is_tape_shaped(c)
            ],
            key=cx
        )
        if not candidates:
            return None, 'none'

        left_tape = None
        right_tape = None
        frame_cx = width / 2.0

        # Temporal association for Near Band (track continuity across frames)
        if not is_far and len(candidates) >= 2:
            if self.prev_near_left_x is not None and self.prev_near_right_x is not None:
                max_shift = MAX_ASSOCIATION_SHIFT_FRAC * width
                best = None

                for i in range(len(candidates)):
                    lx = cx(candidates[i])
                    if abs(lx - self.prev_near_left_x) > max_shift:
                        continue

                    for j in range(i + 1, len(candidates)):
                        rx = cx(candidates[j])
                        if abs(rx - self.prev_near_right_x) > max_shift:
                            continue

                        cost = abs(lx - self.prev_near_left_x) + abs(rx - self.prev_near_right_x)
                        if best is None or cost < best[0]:
                            best = (cost, candidates[i], candidates[j])

                if best is not None:
                    left_tape, right_tape = best[1], best[2]

        if left_tape is None and len(candidates) >= 2:
            # Pick the two largest candidates by area if no temporal match
            by_area = sorted(candidates, key=cv2.contourArea, reverse=True)[:2]
            by_area.sort(key=cx)
            left_tape, right_tape = by_area[0], by_area[1]
        elif len(candidates) == 1:
            only_c = candidates[0]
            only_x = cx(only_c)
            if not is_far and self.prev_near_left_x is not None and self.prev_near_right_x is not None:
                if abs(only_x - self.prev_near_left_x) <= abs(only_x - self.prev_near_right_x):
                    left_tape = only_c
                else:
                    right_tape = only_c
            elif only_x < frame_cx:
                left_tape = only_c
            else:
                right_tape = only_c

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
            lx = cx(left_tape)
            rx = cx(right_tape)
            lane_cx = (lx + rx) / 2.0
            if not is_far:
                self.last_half_gap_px = (rx - lx) / 2.0
                self.single_tape_streak = 0
                self.prev_near_left_x = lx
                self.prev_near_right_x = rx
                self.no_data_streak = 0
            status = 'both'
        elif (
            (left_tape is not None or right_tape is not None)
            and self.last_half_gap_px is not None
            and self.single_tape_streak < MAX_SINGLE_TAPE_STREAK
        ):
            # Dynamic single-tape extrapolation
            visible_tape = left_tape if left_tape is not None else right_tape
            visible_x = cx(visible_tape)
            scale = 0.75 if is_far else 1.0
            gap = self.last_half_gap_px * scale

            if left_tape is not None:
                lane_cx = visible_x + gap
                status = 'left-only'
                if not is_far:
                    self.prev_near_left_x = visible_x
                    self.prev_near_right_x = visible_x + 2.0 * gap
            else:
                lane_cx = visible_x - gap
                status = 'right-only'
                if not is_far:
                    self.prev_near_left_x = visible_x - 2.0 * gap
                    self.prev_near_right_x = visible_x

            # Boundary validation
            if not (0 <= lane_cx <= width):
                if not is_far:
                    self.single_tape_streak = 0
                return None, 'none'

            if not is_far:
                self.single_tape_streak += 1
                self.no_data_streak = 0
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

        # 2. Crop proven horizon-safe ROI (Rows 144 to 240 on 480p)
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

        # 6. Presence Validation & Temporal Memory Reset
        if near_offset is None and far_offset is None:
            self.detected_pub.publish(Bool(data=False))
            self.last_published_offset = None
            self.no_data_streak += 1
            if self.no_data_streak >= NO_DATA_RESET_STREAK:
                self.prev_near_left_x = None
                self.prev_near_right_x = None
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
