from freenove_driver.motor import Ordinary_Car
import time

# Drives straight (nominally - see TRIM below) at several trim values in
# turn, so you can watch each pass and see which value actually tracks
# straight. BASE matches lane_follower.py's BASE_DUTY so whatever TRIM
# value looks straightest here transfers directly to that file's
# MOTOR_TRIM constant.
#
# Positive trim is meant to speed up the left side and slow the right
# side. BASE is NEGATIVE on this chassis (confirmed: this exact value
# drives it forward), and speed is |duty| - so speeding a side up means
# making its duty MORE negative, not less. A first version of this
# script did `left = BASE + trim`, which for negative BASE actually
# makes left LESS negative (slower) as trim increases - backwards from
# the comment above it, and confirmed on-track: every positive trim
# value drifted left as bad as or worse than trim=0, because it was
# slowing the already-weak side down further. Fixed sign below.
BASE = -480
TRIM_VALUES = [0, 30, 60, 90, 120, 150]
DRIVE_TIME_S = 2.0
PAUSE_S = 2.0

car = Ordinary_Car()

try:
    for trim in TRIM_VALUES:
        left = BASE - trim
        right = BASE + trim
        print(f'Testing TRIM={trim}  (left={left}, right={right}) - watch closely')
        car.set_motor_model(left, left, right, right)
        time.sleep(DRIVE_TIME_S)
        car.set_motor_model(0, 0, 0, 0)
        time.sleep(PAUSE_S)
finally:
    car.set_motor_model(0, 0, 0, 0)
    car.close()
