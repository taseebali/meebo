from freenove_driver.motor import Ordinary_Car
import time

# Drives straight (nominally - see TRIM below) at several trim values in
# turn, so you can watch each pass and see which value actually tracks
# straight. BASE matches lane_follower.py's BASE_DUTY so whatever TRIM
# value looks straightest here transfers directly to that file's
# MOTOR_TRIM constant.
#
# Positive trim speeds up the left side and slows the right side -
# matches lane_follower.py's `left = ... + MOTOR_TRIM`,
# `right = ... - MOTOR_TRIM`. A prior run of this script with fully
# symmetric duty (car.set_motor_model(BASE, BASE, BASE, BASE), zero
# trim) confirmed the robot curves LEFT on its own, so the correct
# value should be positive - this sweep is to find how much.
BASE = -600
TRIM_VALUES = [0, 30, 60, 90, 120, 150]
DRIVE_TIME_S = 2.0
PAUSE_S = 2.0

car = Ordinary_Car()

try:
    for trim in TRIM_VALUES:
        left = BASE + trim
        right = BASE - trim
        print(f'Testing TRIM={trim}  (left={left}, right={right}) - watch closely')
        car.set_motor_model(left, left, right, right)
        time.sleep(DRIVE_TIME_S)
        car.set_motor_model(0, 0, 0, 0)
        time.sleep(PAUSE_S)
finally:
    car.set_motor_model(0, 0, 0, 0)
    car.close()
