#!/usr/bin/env pybricks-micropython

from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor
from pybricks.parameters import Port
from pybricks.tools import wait
from math import pi

# EV3 and motors
ev3 = EV3Brick()
left_motor = Motor(Port.B)

right_motor = Motor(Port.C)

# Calibrated values
r = 2.74       # wheel radius in cm
W = 10.96      # effective wheel separation in cm

SPEED = 150


# ------------------------------------------------
# Drive straight by a specified distance in cm
# ------------------------------------------------
def drive_straight(distance_cm):

    # Number of wheel rotations
    rotations = distance_cm / (2 * pi * r)

    # Convert rotations to motor degrees
    degrees = rotations * 360

    left_motor.run_angle(SPEED, degrees, wait=False)
    right_motor.run_angle(SPEED, degrees, wait=True)


# ------------------------------------------------
# Turn robot by a specified angle in degrees
# ------------------------------------------------
def turn(angle_deg):

    # Motor rotations needed for an in-place turn
    degrees = (W * angle_deg / (2 * r))

    left_motor.run_angle(SPEED, degrees, wait=False)
    right_motor.run_angle(SPEED, -degrees, wait=True)


# ------------------------------------------------
# MAIN PROGRAM: 50 cm SQUARE
# ------------------------------------------------

ev3.speaker.beep()

drive_straight(50)
turn(90)

drive_straight(50)
turn(90)

drive_straight(50)
turn(90)

drive_straight(50)
turn(90)

ev3.speaker.beep()

wait(1000)