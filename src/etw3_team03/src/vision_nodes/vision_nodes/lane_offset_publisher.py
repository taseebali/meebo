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
# offset reflects it.
#
# Reverted to 120-280 (out of a 480-tall frame) - this is the region
# that was actually confirmed working on-track, from right after the
# classification fix + 640x480 resolution change. A later attempt to
# trim this further (to 190-280, then 160-300) to dodge tape merging
# on sharp turns never actually got back to this original working
# region - it drifted to a different band instead. Reverting cleanly
# rather than layering another guess on top.
# Tuned against a LIVE frame captured with the robot actually sitting
# on the lane, which is the only way this has been picked reliably -
# earlier attempts tuned against stale saved frames from a different
# robot position and produced bands that were wrong for the real
# geometry.
#
# The two tapes diverge steeply as they approach the robot: in the
# live frame the left tape has already exited the left edge by row
# ~270, and the right by row ~400. So a low "floor-only" band sees no
# tape at all - a 288-448 band found ZERO usable contours on that
# frame, which is exactly the "No usable lane data" spam observed
# on-track. Conversely, bands reaching above row ~180 pull in horizon
# clutter (a shoe at the right edge got picked as "right tape" at
# x=781, giving a +0.133 offset when the true value was about -0.16;
# a 120-280 band put BOTH picks on the right side for a nonsense
# +0.708).
#
# 180-300 (of 600) is the band that cleanly contains both tapes and
# nothing else. Cross-checked against the neighbouring 200-320 band,
# which independently agrees (-0.162 vs -0.152) - that agreement is
# the evidence this is measuring the real lane and not an artifact.
# Verified visually too: both selected contours outline actual tape.
ROI_TOP_FRAC = 144 / 480
ROI_BOTTOM_FRAC = 240 / 480

# Minimum contour area to trust as "this is a lane line," as a
# fraction of the ROI band's total pixel area (width * height) rather
# than a fixed pixel count - so it scales automatically with whatever
# resolution/ROI size is actually in effect. Reverted to the value
# that matched the working 120-280 band (130px on a 160-row x 640-col
# band), for the same reason as the ROI revert above.
MIN_CONTOUR_AREA_FRAC = 130 / (160 * 640)

# Maximum contour bounding-box width to trust as "this is a lane line,"
# as a fraction of the ROI band's width. Confirmed via a saved frame
# (frame_135.png) that the ROI's HSV mask isn't tape-exclusive - a
# shadow/clutter patch in the same brightness range as the tape (window
# glare makes the tape's own V swing 50-184, overlapping the shadow's
# 106-144, so V-thresholding alone can't separate them) produced a
# contour with bbox width ~524px, dwarfing the real tape contours
# (~82-132px wide in that same frame) and winning the area-based
# top-2 selection outright - both "tapes" ended up being pieces of the
# same non-tape blob. A real tape segment in this ROI is a narrow
# strip; anything wider than this is more likely a room/floor
# artifact than tape, regardless of how much area it has.
MAX_CONTOUR_WIDTH_FRAC = 0.25

# Sanity check applied AFTER picking the two largest tape-shaped
# contours: reject the pair if they don't actually straddle the frame
# center. This is NOT the classification method (that's still the
# relative-position ordering above, which is what makes real curves
# work) - it's a guard against two contours from the SAME side/object
# both passing the area+width filters, which still happens (confirmed
# on frame_135.png: two fragments of the same right-side region at
# x=638 and x=766 both survived filtering and would otherwise have
# been accepted as "left"/"right" tape, producing a confidently wrong
# offset instead of falling back to LANE_TIMEOUT_S like a genuine
# no-data frame would). A real curve can shift both tapes well off
# center, so this margin is generous - it only catches "both
# candidates are deep on the same side," not "both leaned the same
# direction."
STRADDLE_MARGIN_FRAC = 0.5

# How many consecutive single-tape frames to bridge with an estimated
# lane center (see single-tape handling below) before giving up and
# falling back to "no data." Frames arrive at roughly the camera's
# processing rate here, not a fixed FPS, so this is an approximate
# cap, not a precise time bound - kept short deliberately since it's
# extrapolating from a possibly-stale gap measurement.
MAX_SINGLE_TAPE_STREAK = 15

# Logging every frame (at full camera rate) is expensive on a Pi and was
# eating into the time available for actual frame processing, adding
# latency to the offset the lane follower reacts to. Throttle it.
LOG_EVERY_N = 15

# Dilation kernel applied to the mask before contour detection. On a
# sharp turn, live testing showed the camera catching real motion blur
# - the tape thins out and sometimes breaks into fragments under blur,
# which drops below MIN_CONTOUR_AREA and causes total lane-data loss
# (confirmed: 2.6 seconds of zero detections on one sharp corner,
# car sitting stopped the whole time). Dilating thickens/bridges thin
# or fragmented mask regions before they're filtered by area, so a
# blurred tape is more likely to still register as one valid contour
# instead of vanishing entirely.
DILATE_KERNEL = np.ones((5, 5), np.uint8)

# Reject a single-frame offset jump this large unless the SAME jump
# repeats on the next frame too. Confirmed on-track: raw_offset flipped
# from +0.115 (lane right, steer right) to -0.721 (lane left, steer
# left) in under 0.7s (one frame), which is physically impossible for
# a real curve at driving speed but was enough on its own to slam the
# steering clamp and drive the robot off the track before vision could
# recover (frame_060 still on-track, frame_065 already lost the left
# tape from view). A real curve produces the same large offset on
# consecutive frames; a bad detection usually doesn't - so requiring
# one frame of agreement before trusting a big jump filters out the
# spike without meaningfully delaying a genuine curve.
MAX_OFFSET_JUMP = 0.35


class LaneOffsetPublisher(Node):

    def __init__(self):
        super().__init__('lane_offset_publisher')

        self.publisher_ = self.create_publisher(
            Float32,
            'lane_offset',
            10
        )

        # Depth 1, not 10: if this node ever falls a frame behind the
        # camera, a deeper queue lets stale frames pile up and get
        # processed in order, so the offset we publish reflects where
        # the lane was several frames ago instead of now - on a curve
        # that's the difference between reacting in time and reacting
        # after the robot has already crossed the tape. Depth 1 always
        # drops old frames in favor of the newest one.
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

        # Thicken/bridge thin or fragmented regions (e.g. from motion
        # blur on a sharp turn) before contour detection - see
        # DILATE_KERNEL comment above.
        mask = cv2.dilate(mask, DILATE_KERNEL, iterations=1)

        # Calculate center of image
        frame_center_x = roi.shape[1] / 2.0

        # The track is TWO separate tape lines, not one - the robot
        # should track the MIDPOINT between them, not center itself on
        # either tape individually (which just makes it hug one edge
        # of the lane).
        #
        # Classifying contours by a FIXED frame center ("left of
        # center" / "right of center") breaks down on a sharp turn: if
        # the robot's heading swings enough, BOTH tapes can end up on
        # the same side of that fixed center, and the classification
        # flips unpredictably frame to frame - confirmed on-track
        # (raw_offset whipsawing between -0.4 and +0.24 within ~1.5s
        # during a sharp turn, which a real lane offset never does).
        # Instead, take the two largest significant contours and label
        # them "left"/"right" by their position RELATIVE TO EACH
        # OTHER, not relative to a fixed center - this holds up even
        # when the whole lane has shifted across the frame.
        contours, _ = cv2.findContours(
            mask,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE
        )

        def contour_center_x(contour):
            m = cv2.moments(contour)
            return m['m10'] / m['m00']

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
            # Both tapes visible - track the midpoint between them
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
            # Exactly one tape visible - previously this always fell
            # back to "no data," but the 2026-08-14 test showed that
            # can mean losing ALL steering input for 20+ seconds
            # straight during a turn (one tape genuinely out of frame/
            # ROI for that whole stretch), well past LANE_TIMEOUT_S's
            # 1.5s grace period, so the bot just sat stopped mid-turn
            # instead of continuing to correct. Bridge short gaps using
            # the gap width measured from the LAST successful two-tape
            # frame (not a fixed guess - avoids the earlier bias issue
            # from a hardcoded HALF_LANE_WIDTH_PX) to estimate where
            # the missing tape should be. Bounded by
            # MAX_SINGLE_TAPE_STREAK so a stale gap measurement can't
            # be trusted indefinitely - beyond that, fall through to
            # the same "no data" path as before and let LANE_TIMEOUT_S
            # do its job as the real safety net.
            visible_tape = left_tape if left_tape is not None else right_tape
            visible_x = contour_center_x(visible_tape)
            estimated_center_x = (
                visible_x + self.last_half_gap_px if left_tape is not None
                else visible_x - self.last_half_gap_px
            )

            # The gap it's extrapolating from could itself be from a
            # bad two-tape detection (confirmed on-track: a
            # last_half_gap_px poisoned this way produced an estimate
            # landing outside the frame entirely - offset candidate of
            # 1.783, mathematically impossible for a real in-frame
            # contour). If the estimate isn't even inside the ROI,
            # it's not usable - discard it the same as "no data"
            # rather than publishing something worse than a plain
            # missing reading.
            if not (0 <= estimated_center_x <= roi.shape[1]):
                self.single_tape_streak = 0
                if self.frame_count % LOG_EVERY_N == 0:
                    self.get_logger().warn(
                        f'Discarding single-tape estimate '
                        f'({estimated_center_x:.1f}) - falls outside '
                        f'the frame, last_half_gap_px is untrustworthy'
                    )
                return

            lane_center_x = estimated_center_x
            self.single_tape_streak += 1
            if self.frame_count % LOG_EVERY_N == 0:
                self.get_logger().warn(
                    f'Only one tape visible - estimating lane center '
                    f'from last known gap width '
                    f'(streak={self.single_tape_streak}/'
                    f'{MAX_SINGLE_TAPE_STREAK})'
                )
        else:
            self.single_tape_streak = 0
            if self.frame_count % LOG_EVERY_N == 0:
                self.get_logger().warn(
                    'No usable lane data - treating as no lane data'
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

        # Outlier rejection - see MAX_OFFSET_JUMP above. Don't trust a
        # big single-frame jump from the last PUBLISHED offset unless
        # the same jump shows up again next frame.
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
                if self.frame_count % LOG_EVERY_N == 0:
                    self.get_logger().warn(
                        f'Rejected offset jump: last_published='
                        f'{self.last_published_offset:.3f} '
                        f'candidate={offset:.3f} - waiting for next '
                        f'frame to confirm before trusting it'
                    )
                return
        else:
            self.pending_offset = None
            self.pending_count = 0

        self.last_published_offset = offset

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
