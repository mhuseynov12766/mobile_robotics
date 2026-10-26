
#!/usr/bin/env pybricks-micropython

from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, ColorSensor
from pybricks.parameters import Port
from pybricks.robotics import DriveBase
from pybricks.tools import wait

# Initialize the EV3 brick
ev3 = EV3Brick()

# Initialize drive motors
LEFT_MOTOR = Motor(Port.B)
RIGHT_MOTOR = Motor(Port.C)

# Initialize the color sensor
LINE_SENSOR = ColorSensor(Port.S3)

# Set up the robot's dimensions
WHEEL_DIAMETER = 55
AXLE_TRACK = 110

robot = DriveBase(
    LEFT_MOTOR,
    RIGHT_MOTOR,
    WHEEL_DIAMETER,
    AXLE_TRACK
)

# Calibration values measured from the lab floor and tape
BLACK_REFLECTION = 7.5
WHITE_REFLECTION = 62

# Calculate the target threshold
TARGET_THRESHOLD = (
    BLACK_REFLECTION + WHITE_REFLECTION
) / 2

# PD controller parameters
DRIVE_SPEED = 170       # Forward speed in mm/s
PROPORTIONAL_GAIN = 1.8
DERIVATIVE_GAIN = 2.0
MAX_TURN_RATE = 130     # Maximum turn rate in deg/s

# Line-loss recovery parameters
LOST_LINE_TOLERANCE = 0
LOST_LINE_COUNT_MAX = 50
TOTAL_MAX_SEARCHES = 13
RIGHT_SEARCH_LIMIT = 120
TOTAL_SEARCH_LIMIT = 600
SEARCH_TURN_RATE = 50


def search_for_line(mission_lost_count):
    """Search for the line when the robot loses it."""
    robot.stop()
    ev3.speaker.beep(100, 100)

    SEARCH_SPEED = 0

    if mission_lost_count < TOTAL_MAX_SEARCHES:

        # Search towards the right
        initial_angle = LEFT_MOTOR.angle()

        while LEFT_MOTOR.angle() - initial_angle < RIGHT_SEARCH_LIMIT:
            robot.drive(SEARCH_SPEED, SEARCH_TURN_RATE)

            if LINE_SENSOR.reflection() <= (
                TARGET_THRESHOLD - LOST_LINE_TOLERANCE
            ):
                robot.stop()
                ev3.speaker.beep(1000, 100)
                wait(500)
                return mission_lost_count + 1

        # Sweep towards the left
        initial_angle = LEFT_MOTOR.angle()

        while initial_angle - LEFT_MOTOR.angle() < TOTAL_SEARCH_LIMIT:
            robot.drive(SEARCH_SPEED, -SEARCH_TURN_RATE)

            if LINE_SENSOR.reflection() <= (
                TARGET_THRESHOLD - LOST_LINE_TOLERANCE
            ):
                robot.stop()
                ev3.speaker.beep(1000, 100)
                wait(500)
                return mission_lost_count + 1

        return mission_lost_count + 1

    else:
        # Continue searching until the line is found
        while True:
            initial_angle = LEFT_MOTOR.angle()

            while LEFT_MOTOR.angle() - initial_angle < TOTAL_SEARCH_LIMIT:
                robot.drive(SEARCH_SPEED, -SEARCH_TURN_RATE)

                if LINE_SENSOR.reflection() <= (
                    TARGET_THRESHOLD - LOST_LINE_TOLERANCE
                ):
                    robot.stop()
                    ev3.speaker.beep(1000, 100)
                    wait(500)
                    return TOTAL_MAX_SEARCHES

            wait(500)


def run_mission():
    """Run PD line following with line-loss recovery."""
    ev3.speaker.beep()

    lost_line_counter = 0
    mission_lost_count = 0
    last_error = 0

    robot.drive(DRIVE_SPEED, 0)

    while True:
        # Read the current color sensor reflection
        current_reflection = LINE_SENSOR.reflection()

        # Detect possible line loss
        if current_reflection > (
            TARGET_THRESHOLD + LOST_LINE_TOLERANCE
        ):
            lost_line_counter += 1

            if lost_line_counter > LOST_LINE_COUNT_MAX:
                mission_lost_count = search_for_line(
                    mission_lost_count
                )
                lost_line_counter = 0
                last_error = 0
        else:
            lost_line_counter = 0

        # Calculate the PD controller error
        error = current_reflection - TARGET_THRESHOLD
        derivative = error - last_error

        # Calculate the steering turn rate
        turn_rate = (
            PROPORTIONAL_GAIN * error
            + DERIVATIVE_GAIN * derivative
        )

        # Limit the turn rate
        if turn_rate > MAX_TURN_RATE:
            turn_rate = MAX_TURN_RATE
        elif turn_rate < -MAX_TURN_RATE:
            turn_rate = -MAX_TURN_RATE

        # Drive forward while correcting the steering
        robot.drive(DRIVE_SPEED, turn_rate)

        # Store the error for the next iteration
        last_error = error

        # Short delay between control updates
        wait(10)


if __name__ == "__main__":
    run_mission()
