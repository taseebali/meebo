"""
Tuning helper for lane_offset_publisher_hough.py — run this against a saved
frame (from frame_saver) instead of the live ROS node so you can iterate in
seconds instead of rebuilding/relaunching every time.

This mirrors lane_offset_publisher_hough.py's actual pipeline exactly
(same Canny -> ROI -> HoughLinesP -> classify -> weighted average steps),
so whatever values you land on here are the values to paste into the real
node — this script isn't a simplified approximation of it.

Usage:
    python3 tune_hough.py path/to/frame.png

Writes three debug images next to the input frame:
    <frame>_edges.png    - the raw Canny edge output (before ROI masking)
    <frame>_roi.png       - edges after the ROI trapezoid is applied
    <frame>_hough.png     - detected lines drawn on the original frame,
                             color-coded: blue = classified left,
                             red = classified right, gray = filtered out
                             (too horizontal, or on the wrong side)

Prints the same numbers the real node would compute for this frame,
including the measured lane width when both sides are found - use that
number for ASSUMED_LANE_WIDTH_PX in the real node, don't guess it.
"""
import sys
import cv2
import numpy as np

# ============ TUNE THESE ON YOUR TRACK ============
# Same constants as lane_offset_publisher_hough.py - copy your final values
# back into that file once you're happy with what you see here.
CANNY_LOW = 50
CANNY_HIGH = 150

ROI_TOP_Y = 0.5
ROI_TOP_LEFT_X = 0.35
ROI_TOP_RIGHT_X = 0.65

MIN_LINE_SLOPE = 0.4
EVAL_Y_FRACTION = 0.9
# ====================================================


def classify_and_locate(lines, width, height):
    """Identical logic to lane_offset_publisher_hough.py - returns
    (left_x, right_x, left_lines, right_lines, rejected_lines) so this
    script can both compute the result and draw what happened."""
    if lines is None or len(lines) == 0:
        return None, None, [], [], []

    image_center = width / 2.0
    left_lines = []
    right_lines = []
    rejected_lines = []

    for line in lines:
        x1, y1, x2, y2 = np.asarray(line).reshape(-1)[:4]
        if x2 - x1 == 0:
            rejected_lines.append((x1, y1, x2, y2))
            continue
        slope = (y2 - y1) / (x2 - x1)
        if abs(slope) < MIN_LINE_SLOPE:
            rejected_lines.append((x1, y1, x2, y2))
            continue
        center_x = (x1 + x2) / 2.0
        if slope < 0 and center_x < image_center * 1.2:
            left_lines.append((x1, y1, x2, y2))
        elif slope > 0 and center_x > image_center * 0.8:
            right_lines.append((x1, y1, x2, y2))
        else:
            rejected_lines.append((x1, y1, x2, y2))

    eval_y = height * EVAL_Y_FRACTION
    left_x = weighted_x_at(left_lines, eval_y)
    right_x = weighted_x_at(right_lines, eval_y)
    return left_x, right_x, left_lines, right_lines, rejected_lines


def weighted_x_at(lines, eval_y):
    if not lines:
        return None
    weighted_sum = 0.0
    total_weight = 0.0
    for x1, y1, x2, y2 in lines:
        length = np.hypot(x2 - x1, y2 - y1)
        if abs(y2 - y1) > 1:
            t = (eval_y - y1) / (y2 - y1)
            x_at_eval = x1 + t * (x2 - x1)
        else:
            x_at_eval = (x1 + x2) / 2.0
        weighted_sum += x_at_eval * length
        total_weight += length
    return weighted_sum / total_weight if total_weight > 0 else None


def main():
    path = sys.argv[1] if len(sys.argv) > 1 else 'frame_000.png'
    frame = cv2.imread(path)
    if frame is None:
        raise SystemExit(f'Could not read {path}')

    height, width = frame.shape[:2]
    print(f'{path}  shape={frame.shape}')

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur(gray, (5, 5), 0)
    edges = cv2.Canny(blur, CANNY_LOW, CANNY_HIGH)
    edges_path = path.replace('.png', '_edges.png').replace('.jpg', '_edges.png')
    cv2.imwrite(edges_path, edges)
    print(f'Wrote {edges_path}  edge pixels: {int((edges > 0).sum())}')

    roi_vertices = np.array([[
        (width * 0.0, height),
        (width * ROI_TOP_LEFT_X, height * ROI_TOP_Y),
        (width * ROI_TOP_RIGHT_X, height * ROI_TOP_Y),
        (width * 1.0, height),
    ]], dtype=np.int32)
    roi_mask = np.zeros_like(edges)
    cv2.fillPoly(roi_mask, roi_vertices, 255)
    roi_edges = cv2.bitwise_and(edges, roi_mask)
    roi_path = path.replace('.png', '_roi.png').replace('.jpg', '_roi.png')
    cv2.imwrite(roi_path, roi_edges)
    print(f'Wrote {roi_path}  edge pixels inside ROI: {int((roi_edges > 0).sum())}')

    lines = cv2.HoughLinesP(
        roi_edges, rho=1, theta=np.pi / 180,
        threshold=40, minLineLength=40, maxLineGap=150)
    num_lines = 0 if lines is None else len(lines)
    print(f'HoughLinesP found {num_lines} raw segments')

    left_x, right_x, left_lines, right_lines, rejected_lines = \
        classify_and_locate(lines, width, height)

    debug = frame.copy()
    cv2.polylines(debug, roi_vertices, isClosed=True, color=(0, 255, 255), thickness=2)
    for x1, y1, x2, y2 in rejected_lines:
        cv2.line(debug, (x1, y1), (x2, y2), (128, 128, 128), 1)
    for x1, y1, x2, y2 in left_lines:
        cv2.line(debug, (x1, y1), (x2, y2), (255, 0, 0), 3)
    for x1, y1, x2, y2 in right_lines:
        cv2.line(debug, (x1, y1), (x2, y2), (0, 0, 255), 3)

    eval_y = int(height * EVAL_Y_FRACTION)
    frame_center_x = width / 2.0
    cv2.line(debug, (int(frame_center_x), 0), (int(frame_center_x), height), (0, 255, 0), 1)

    print(f'left lines: {len(left_lines)}  right lines: {len(right_lines)}  rejected: {len(rejected_lines)}')

    if left_x is not None:
        cv2.circle(debug, (int(left_x), eval_y), 8, (255, 0, 0), -1)
        print(f'left_x  = {left_x:.1f}')
    if right_x is not None:
        cv2.circle(debug, (int(right_x), eval_y), 8, (0, 0, 255), -1)
        print(f'right_x = {right_x:.1f}')

    if left_x is not None and right_x is not None:
        lane_center_x = (left_x + right_x) / 2.0
        measured_width = right_x - left_x
        print(f'BOTH sides found. Measured lane width = {measured_width:.1f}px '
              f'-> use this for ASSUMED_LANE_WIDTH_PX')
    elif left_x is not None:
        lane_center_x = left_x + 130.0  # placeholder half-width until you set ASSUMED_LANE_WIDTH_PX for real
        print('Only LEFT found - using a placeholder half-width for this preview. '
              'Set ASSUMED_LANE_WIDTH_PX from a both-sides frame first.')
    elif right_x is not None:
        lane_center_x = right_x - 130.0
        print('Only RIGHT found - using a placeholder half-width for this preview. '
              'Set ASSUMED_LANE_WIDTH_PX from a both-sides frame first.')
    else:
        lane_center_x = None
        print('NO lane lines found in this frame.')

    if lane_center_x is not None:
        offset = (lane_center_x - frame_center_x) / frame_center_x
        cv2.circle(debug, (int(lane_center_x), eval_y), 8, (0, 255, 0), -1)
        print(f'lane_center_x = {lane_center_x:.1f}  frame_center_x = {frame_center_x:.1f}  '
              f'offset = {offset:.3f}')

    debug_path = path.replace('.png', '_hough.png').replace('.jpg', '_hough.png')
    cv2.imwrite(debug_path, debug)
    print(f'Wrote {debug_path}  (blue=left, red=right, gray=rejected, '
          f'green line=frame center, green dot=computed lane center)')


if __name__ == '__main__':
    main()
