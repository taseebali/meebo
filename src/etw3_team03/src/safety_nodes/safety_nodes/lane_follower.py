import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

from freenove_driver.motor import Ordinary_Car

STOP_DISTANCE_CM = 65
WATCHDOG_TIMEOUT_S = 3.0

# Lowered from 900: slower driving gives the vision/control loop more
# time to react per unit distance travelled (shorter physical stopping
# distance too), and makes the steering clamp below a much bigger
# fraction of drive speed, so turns are sharper at the same clamp.
BASE_DUTY = 600

# Fixed hardware trim for a physical left/right motor-strength mismatch
# on this chassis - confirmed independent of vision/steering via
# steering_direction_characterization.py: commanding all four wheels
# identically (car.set_motor_model(BASE, BASE, BASE, BASE), no bias of
# any kind) still curves the robot LEFT. That means the left side is
# physically weaker than the right, and no amount of PD tuning on
# lane_offset can fix it - the control loop only reacts to an offset
# AFTER it appears, but this bias is present even at adjustment=0, on
# a straight, with a correctly-centered lane_offset. Applied as a
# constant added to the left side's duty and subtracted from the
# right's, on top of (not instead of) the PD steering adjustment, so
# it cancels the physical bias while curve-following is unaffected.
#
# Starting value only - re-run steering_direction_characterization.py
# (now sweeps a range of trim values one at a time) and watch which
# value actually drives straightest, then update this constant to
# match. Sign convention: positive MOTOR_TRIM speeds up the left side
# and slows the right side, matching the direction needed to correct
# the confirmed left drift.
MOTOR_TRIM = 40

# Proportional steering gain
# KP=4.0 (with the +/-500 clamp below) was confirmed too aggressive
# via video of an actual test run: on a real curve it didn't turn
# proportionally, it slammed into a near-max rotation, overshot past
# facing the right way, then swung back the other way - visible as
# the car spinning ~90 degrees and rocking back rather than smoothly
# following the bend. That's oscillation from too much gain, not a
# wrong-direction bug (the sign was verified correct/consistent
# throughout the same test's logs). Backed off to reduce overshoot.
#
# Raised again from 2.0 for the 2026-08-13 test: video from that run
# showed the opposite problem - on a real curve the bot tracked
# straight lines fine but turned too weakly and late to actually
# follow the bend (rode up onto/over the outer tape instead of
# curving with it). KP=2.0 was tuned back when there was no
# derivative term and a tight MAX_ADJUSTMENT_STEP doing the
# anti-oscillation work below - now that both KD and a looser rate
# cap share that job, KP can push harder without reintroducing the
# KP=4.0 spin-and-rock behavior. Needs re-confirming on the next
# physical run.
KP = 2.8

# Derivative gain - reacts to how FAST lane_offset is changing, not
# just how far off-center it currently is. A pure-P controller only
# starts correcting once the offset has already grown, which is what
# made the bot look like it "found out" about a curve only after
# partway through it (by the time a big enough offset had built up
# to produce a real correction, the curve was already underway). The
# derivative term produces a correction as soon as the offset starts
# moving, before it's large - earlier response to the same curve.
# Untested value - start conservative and increase if curves are
# still taken too late/too softly, decrease if the bot starts
# reacting jerkily to normal frame-to-frame offset noise.
KD = 0.15

# Smoothing applied to the derivative term specifically (separate
# from OFFSET_SMOOTHING above). A simulated-offset check while
# building this showed that with OFFSET_SMOOTHING=0.8 barely
# filtering anything, plain frame-to-frame sensor noise - the same
# noise CENTER_TOLERANCE exists to ignore on the P term - gets
# multiplied by 1/0.05=20x by the derivative and reintroduces
# straight-line twitching that the dead-band doesn't catch (the
# dead-band only zeroes the P error, not D). This EMA smooths that
# noise out while still responding within 1-2 ticks to a real,
# sustained ramp like a curve entry.
DERIVATIVE_SMOOTHING = 0.4

# Positive lane_offset means the lane is detected to the right of frame
# center, which means the car needs to steer right to re-center on it.
STEER_SIGN = -1

# Target for lane_offset when the robot is actually centered in the
# lane. Left at 0.0 (the geometric frame center) since that's what
# this codebase has been tuned against - a teammate's separate lane
# follower found their robot's true center sat at +0.12 instead, so
# if the next physical test shows a similar consistent bias, recenter
# this rather than the raw offset.
CENTER_TARGET = 0.0

# Error smaller than this is treated as "close enough to center,
# don't correct" rather than fed through KP - stops the bot hunting/
# twitching in response to small frame-to-frame offset noise while
# driving a straight section. Small enough to not mask the start of
# a real curve.
CENTER_TOLERANCE = 0.03

# Stop if we haven't received a lane offset recently
# Raised from 1.0: live testing showed a sharp turn causing a real,
# multi-second vision dropout (motion blur + tapes converging near
# the vanishing point - see mask dilation fix in
# lane_offset_publisher.py, which targets the root cause). This gives
# a bit more grace before a full stop for whatever gap remains after
# that fix, without weakening the timeout as a genuine safety net for
# sustained lane loss.
LANE_TIMEOUT_S = 1.5

# Exponential smoothing on lane_offset (0 < x <= 1, 1 = no smoothing).
# Raised from 0.6 - the two-tape midpoint signal is cleaner than the
# old single-contour one, so less filtering is needed and the extra
# lag isn't worth it anymore.
OFFSET_SMOOTHING = 0.8

# Max change in `adjustment` allowed per control_loop tick (runs at
# 20Hz - see timer below). On-track testing showed a single bad/
# ambiguous vision frame could snap the commanded adjustment straight
# from one clamp extreme to the other (+500 to -500) in about a
# second, which threw the robot off the marked path at a sharp turn
# instead of tracking it. This limits how fast the commanded steering
# can move regardless of how much the input signal jumps - real turns
# still get there, just ramped instead of snapped.
#
# Raised from 60 for the 2026-08-13 test: at 60/tick (20Hz), reaching
# the +/-300 clamp from zero takes 5 ticks (~250ms) no matter how far
# off-center the offset says the bot is - video showed this as the
# bot barely starting to turn before it was already past the curve.
# 60 was sized back when lane_offset had no smoothing at all and a
# single glitchy frame could hit the clamp directly; OFFSET_SMOOTHING
# below now does most of that noise-rejection job, so the rate cap
# can be loosened to let real, sustained offset changes (an actual
# curve) reach full correction in ~1 tick instead of 5, while still
# catching a single-frame outlier before it becomes a full snap.
MAX_ADJUSTMENT_STEP = 240



class LaneFollower(Node):

    def __init__(self):
        super().__init__('lane_follower')

        self.car = Ordinary_Car()

        self.last_distance_time = None
        self.last_distance = None

        self.last_offset_time = None
        self.last_offset = 0.0
        self.prev_offset_for_derivative = 0.0
        self.smoothed_derivative = 0.0

        self.offset_msg_count = 0
        self.last_adjustment = 0.0

        self.create_subscription(
            Float32,
            'distance_cm',
            self.on_distance,
            10
        )

        # Depth 1: always act on the newest lane_offset, never a
        # queued-up stale one - same reasoning as the image
        # subscription depth in lane_offset_publisher.py.
        self.create_subscription(
            Float32,
            'lane_offset',
            self.on_offset,
            1
        )

        # Faster control loop (was 0.1s/10Hz) - reacts to the latest
        # received offset/distance sooner instead of sitting on stale
        # values for up to 100ms between checks.
        self.create_timer(0.05, self.control_loop)

        self.stop_motors()

        self.get_logger().info('Lane follower started')

    def on_distance(self, msg):
        self.last_distance_time = time.monotonic()
        self.last_distance = msg.data

    def on_offset(self, msg):
        self.last_offset_time = time.monotonic()
        raw = msg.data
        self.last_offset = (
            OFFSET_SMOOTHING * raw
            + (1 - OFFSET_SMOOTHING) * self.last_offset
        )
        self.offset_msg_count += 1
        if self.offset_msg_count % 5 == 0:
            self.get_logger().info(
                f'DEBUG: raw_offset={raw:.3f} '
                f'smoothed_offset={self.last_offset:.3f} '
                f'({"lane right of center, should steer RIGHT" if raw > 0 else "lane left of center, should steer LEFT"})'
            )

    def control_loop(self):

        # -----------------------------
        # SAFETY: no distance received
        # -----------------------------
        if self.last_distance_time is None:
            self.stop_motors()
            return

        # -----------------------------
        # SAFETY: distance data stale
        # -----------------------------
        if time.monotonic() - self.last_distance_time > WATCHDOG_TIMEOUT_S:
            self.get_logger().warn(
                'Distance watchdog timeout - stopping motors'
            )
            self.stop_motors()
            return

        # -----------------------------
        # SAFETY: obstacle too close
        # -----------------------------
        if self.last_distance < STOP_DISTANCE_CM:
            self.stop_motors()
            return

        # -----------------------------
        # SAFETY: no lane data
        # -----------------------------
        if self.last_offset_time is None:
            self.stop_motors()
            return

        # -----------------------------
        # SAFETY: lane data stale
        # -----------------------------
        if time.monotonic() - self.last_offset_time > LANE_TIMEOUT_S:
            self.get_logger().warn(
                'Lane offset timeout - stopping motors'
            )
            self.stop_motors()
            return

        # -----------------------------
        # PD STEERING
        # -----------------------------
        error = self.last_offset - CENTER_TARGET

        if abs(error) < CENTER_TOLERANCE:
            error = 0.0

        # Rate of change of the offset since the last tick. Timer
        # runs at a fixed 20Hz (see create_timer below), so a fixed
        # dt matching that period is used rather than measuring wall
        # time - simpler, and immune to jitter in when lane_offset
        # messages happen to arrive.
        raw_derivative = (self.last_offset - self.prev_offset_for_derivative) / 0.05
        self.prev_offset_for_derivative = self.last_offset

        self.smoothed_derivative = (
            DERIVATIVE_SMOOTHING * raw_derivative
            + (1 - DERIVATIVE_SMOOTHING) * self.smoothed_derivative
        )

        target_adjustment = (
            STEER_SIGN
            * (KP * error + KD * self.smoothed_derivative)
            * BASE_DUTY
        )

        # Lowered from 500 alongside the KP reduction above - at 500 a
        # single sharp reading could still hit near-max rotation even
        # with lower gain. 300 (half of BASE_DUTY=600) still lets the
        # inner wheel slow to a near-stall on a real hard turn, without
        # the same overshoot magnitude that caused the spin-and-rock
        # behavior seen on video.
        target_adjustment = max(-300, min(300, target_adjustment))

        # Rate-limit: move last_adjustment toward target_adjustment by
        # at most MAX_ADJUSTMENT_STEP this tick, instead of jumping
        # straight to it.
        step = max(
            -MAX_ADJUSTMENT_STEP,
            min(MAX_ADJUSTMENT_STEP, target_adjustment - self.last_adjustment)
        )
        adjustment = self.last_adjustment + step
        self.last_adjustment = adjustment

        left = BASE_DUTY - adjustment + MOTOR_TRIM
        right = BASE_DUTY + adjustment - MOTOR_TRIM

        left = int(left)
        right = int(right)

        if self.offset_msg_count % 5 == 0:
            self.get_logger().info(
                f'DEBUG: adjustment={adjustment:.1f} '
                f'left_duty={left} right_duty={right} '
                f'({"left wheels faster -> turning RIGHT" if left > right else "right wheels faster -> turning LEFT"})'
            )

        self.car.set_motor_model(
            -left,
            -left,
            -right,
            -right
        )

    def stop_motors(self):
        self.car.set_motor_model(0, 0, 0, 0)
        self.last_adjustment = 0.0
        # Also reset the derivative's reference point, so resuming
        # after a safety stop doesn't compute a derivative spike from
        # whatever the offset happened to do while stopped.
        self.prev_offset_for_derivative = self.last_offset
        self.smoothed_derivative = 0.0

    def destroy_node(self):
        self.get_logger().info('Stopping motors and shutting down')

        try:
            self.stop_motors()
            self.car.close()
        except Exception as e:
            self.get_logger().warn(
                f'Error while closing motor controller: {e}'
            )

        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    node = LaneFollower()

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