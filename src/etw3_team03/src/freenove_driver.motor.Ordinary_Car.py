from freenove_driver.motor import Ordinary_Car
import time

DUTY = 1200   # try a few values — start low, this is a first guess

car = Ordinary_Car()
car.set_motor_model(-DUTY, -DUTY, -DUTY, -DUTY)   # all four wheels forward
time.sleep(1.0)
car.set_motor_model(0, 0, 0, 0)               # stop
car.close()


# reaction time =  57cm * 1 * 0.75 = 42.75 cm/s
