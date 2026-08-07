# Provenance

`motor.py`, `pca9685.py`, `servo.py`, and `ultrasonic.py` in this directory
are vendored from Freenove's example code for the 4WD Smart Car Kit for
Raspberry Pi:

- https://github.com/Freenove/Freenove_4WD_Smart_Car_Kit_for_Raspberry_Pi
- commit `a49db4b9dfa9b7a82d172354cb3b4e0ed64985e5`
- path `Code/Server/{motor,pca9685,servo,ultrasonic}.py`

They are downloaded by `etw3-setup/setup.sh` at provisioning time (not
committed to the `etw3-setup` repo), so the commit pin above is the source
of truth for exactly which version is running on the robots. If you're
reading this file on an actual Pi, those four files should be present as
siblings of this one.

## License

Creative Commons Attribution-NonCommercial-ShareAlike 3.0 Unported
(CC BY-NC-SA 3.0) — https://creativecommons.org/licenses/by-nc-sa/3.0/

This course's use (non-commercial teaching) is within scope. If a team's
GitHub repo is public, it is redistributing this package too — keep this
notice attached, and be aware ShareAlike means any redistribution of it
(or a derivative) needs to stay under the same license. Don't strip this
file out.

## Modification from the original

`motor.py` and `servo.py` originally import PCA9685 with a flat
`from pca9685 import PCA9685`, which only works when all four files sit
loose in the same directory, as Freenove ships them. `setup.sh` rewrites
that one line in each file to a relative import
(`from .pca9685 import PCA9685`) so the files work as a proper Python
package that colcon can build and any ROS 2 node can `import`. Nothing
else is changed — motor channel mapping, PWM duty-cycle math, the PCA9685
I2C address (0x40), and the default ultrasonic GPIO pins (BCM 27 trigger,
BCM 22 echo) are exactly as Freenove wrote them.
