#!/usr/bin/env python3
from freenove_driver.ultrasonic import Ultrasonic
import time

N = 30
readings = []
for i in range(N):
    with Ultrasonic() as sensor:
        d = sensor.get_distance()
    if d is not None:
        readings.append(d)
    time.sleep(0.1)

print(f"{len(readings)}/{N} valid readings")
if readings:
    print(f"min={min(readings):.1f}  max={max(readings):.1f}  "
          f"mean={sum(readings)/len(readings):.1f}  "
          f"spread={max(readings)-min(readings):.1f}")
