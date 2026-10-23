
#!/usr/bin/env pybricks-micropython

from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor
from pybricks.parameters import Port
from pybricks.tools import wait
from math import pi

# EV3 and motors
ev3 = EV3Brick()
left_motor = Motor(Port.A)
right_motor = Motor(Port.D)

# Calibrated values
r = 2.74       # Wheel radius in cm
w = 10.96      # Effective wheel separation in cm

SPEED = 150

# Drive straight for a given distance in cm
def drive_straight(distance):
    # Calculate the wheel rotations needed
    rotations = distance / (2 * pi * r)

    # Convert rotations to motor degrees
    degrees = rotations * 360

    # Run both motors together
    left_motor.run_angle(SPEED, degrees, wait=False)
    right_motor.run_angle(SPEED, degrees, wait=True)

# Turn left or right by a specified angle
def turn(direction, angle=90):
    # Calculate the distance each wheel must travel
    # for a rotation around the robot's centre
    turn_distance = pi * w * angle / 360

    # Convert wheel travel distance to motor degrees
    rotations = turn_distance / (2 * pi * r)
    motor_degrees = rotations * 360

    if direction == "right":
        left_motor.run_angle(SPEED, motor_degrees, wait=False)
        right_motor.run_angle(SPEED, -motor_degrees, wait=True)

    elif direction == "left":
        left_motor.run_angle(SPEED, -motor_degrees, wait=False)
        right_motor.run_angle(SPEED, motor_degrees, wait=True)

    else:
        raise ValueError("Direction must be 'left' or 'right'")

# Main program: drive a 50 cm square
def main():
    ev3.speaker.beep()

    for _ in range(4):
        drive_straight(50)
        turn("right", 90)
        wait(200)

    ev3.speaker.beep()

# Start the program
main()
