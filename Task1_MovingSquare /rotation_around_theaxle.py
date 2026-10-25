#!/usr/bin/env pybricks-micropython

from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor
from pybricks.parameters import Port

# Initialize EV3 Brick and Motors
ev3 = EV3Brick()
left_motor = Motor(Port.B)
right_motor = Motor(Port.C)

# Experimentally measured wheel radius
WHEEL_RADIUS_CM = 2.74

# Motor speed
MOTOR_SPEED = 150

# TEST:
# Left wheel: 5 rotations forward
# Right wheel: 5 rotations backward
# This makes the robot turn in place.

ev3.speaker.beep()

left_motor.run_angle(MOTOR_SPEED, 5 * 360, wait=False)
right_motor.run_angle(MOTOR_SPEED, -5 * 360, wait=True)

ev3.speaker.beep()