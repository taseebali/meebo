from freenove_driver.motor import Ordinary_Car
import time

# Fires exactly one wheel at a time, in duty1/duty2/duty3/duty4 order -
# matching set_motor_model(duty1, duty2, duty3, duty4) in motor.py,
# which internally names them left_upper, left_lower, right_upper,
# right_lower (in that order). Watch which PHYSICAL wheel moves at
# each step and report back the order - that tells us the real
# duty-index-to-physical-wheel mapping, which lane_follower.py assumes
# is (left_upper, left_lower) = left side, (right_upper, right_lower)
# = right side.

STEPS = [
    ('duty1 (motor.py calls this left_upper)', (-800, 0, 0, 0)),
    ('duty2 (motor.py calls this left_lower)', (0, -800, 0, 0)),
    ('duty3 (motor.py calls this right_upper)', (0, 0, -800, 0)),
    ('duty4 (motor.py calls this right_lower)', (0, 0, 0, -800)),
]

car = Ordinary_Car()

try:
    for label, duties in STEPS:
        print(f'Testing {label} - watch which wheel spins')
        car.set_motor_model(*duties)
        time.sleep(2.0)
        car.set_motor_model(0, 0, 0, 0)
        time.sleep(1.5)
finally:
    car.set_motor_model(0, 0, 0, 0)
    car.close()
