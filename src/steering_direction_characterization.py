from freenove_driver.motor import Ordinary_Car
import time

car = Ordinary_Car()
BASE = -1200
BIAS = -400
car.set_motor_model(BASE + 200, BASE - BIAS, BASE - BIAS, BASE - BIAS)
time.sleep(1.0)
car.set_motor_model(0, 0, 0, 0)
car.close()
