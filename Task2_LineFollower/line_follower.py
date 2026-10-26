#!/usr/bin/env pybricks-micropython

from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, ColorSensor, UltrasonicSensor
from pybricks.parameters import Port
from pybricks.robotics import DriveBase
from pybricks.tools import wait

# Initialize the EV3 brick
ev3 = EV3Brick()

# Initialize drive motors (Port B for left, Port C for right)
LEFT_MOTOR = Motor(Port.B)
RIGHT_MOTOR = Motor(Port.C)

# Initialize sensors (Color sensor on Port 3, Ultrasonic on Port 4)
LINE_SENSOR = ColorSensor(Port.S3)
OBSTACLE_SENSOR = UltrasonicSensor(Port.S4)

# Setup robot dimensions for the DriveBase 
WHEEL_DIAMETER = 55
AXLE_TRACK = 110
robot = DriveBase(LEFT_MOTOR, RIGHT_MOTOR, WHEEL_DIAMETER, AXLE_TRACK)

# Calibration values measured from lab floor (62) and tape (~7.5)
BLACK_REFLECTION = 7.5
WHITE_REFLECTION = 62

# Calculate the target threshold (the edge between floor and tape)
TARGET_THRESHOLD = (BLACK_REFLECTION + WHITE_REFLECTION) / 2

# Control parameters (increased speed for faster lap completion)
DRIVE_SPEED = 170       # Base forward speed in mm/s (increased for a faster run)
PROPORTIONAL_GAIN = 1.8 # Kp: proportional gain for error correction
DERIVATIVE_GAIN = 2.0   # Kd: derivative gain to smooth turns and reduce oscillation
MAX_TURN_RATE = 130     # Maximum turn rate limit in deg/s

# Obstacle detection threshold (distance in mm)
STOP_DISTANCE = 125

# Line loss recovery parameters
LOST_LINE_TOLERANCE = 0
LOST_LINE_COUNT_MAX = 50
TOTAL_MAX_SEARCHES = 13
RIGHT_SEARCH_LIMIT = 120
TOTAL_SEARCH_LIMIT = 600
SEARCH_TURN_RATE = 50 

def search_for_line(mission_lost_count):
    """Executes a search pattern to find the line if the robot completely strays off."""
    robot.stop()
    ev3.speaker.beep(100, 100)
    
    SEARCH_SPEED = 0
    
    if mission_lost_count < TOTAL_MAX_SEARCHES:
        # Search towards the right first
        initial_angle = LEFT_MOTOR.angle()
        while LEFT_MOTOR.angle() - initial_angle < RIGHT_SEARCH_LIMIT:
            robot.drive(SEARCH_SPEED, SEARCH_TURN_RATE)
            if LINE_SENSOR.reflection() <= TARGET_THRESHOLD - LOST_LINE_TOLERANCE:
                robot.stop()
                ev3.speaker.beep(1000, 100)
                wait(500)
                return mission_lost_count + 1

        # If not found, sweep towards the left
        initial_angle = LEFT_MOTOR.angle()
        while initial_angle - LEFT_MOTOR.angle() < TOTAL_SEARCH_LIMIT:
            robot.drive(SEARCH_SPEED, -SEARCH_TURN_RATE)
            if LINE_SENSOR.reflection() <= TARGET_THRESHOLD - LOST_LINE_TOLERANCE:
                robot.stop()
                ev3.speaker.beep(1000, 100)
                wait(500)
                return mission_lost_count + 1

        return mission_lost_count + 1
    
    else:
        # Fallback 360 search loop if maximum search failures are reached
        while True:
            initial_angle = LEFT_MOTOR.angle()
            while LEFT_MOTOR.angle() - initial_angle < TOTAL_SEARCH_LIMIT:
                robot.drive(SEARCH_SPEED, -SEARCH_TURN_RATE)
                if LINE_SENSOR.reflection() <= TARGET_THRESHOLD - LOST_LINE_TOLERANCE:
                    robot.stop()
                    ev3.speaker.beep(1000, 100)
                    wait(500)
                    return TOTAL_MAX_SEARCHES
            wait(500)
    
def wait_for_obstacle_removal():
    """Pauses the robot when an obstacle is detected until it is cleared."""
    robot.stop()
    ev3.speaker.beep(500, 100) 
    
    while OBSTACLE_SENSOR.distance() < STOP_DISTANCE:
        wait(200) 
    
    ev3.speaker.beep(1000, 100)
    wait(1000)

def run_mission():
    """Main execution loop for PD line following and obstacle monitoring."""
    ev3.speaker.beep()
    
    lost_line_counter = 0
    mission_lost_count = 0
    last_error = 0
    
    robot.drive(DRIVE_SPEED, 0)

    while True:
        # Check distance to any oncoming obstacle or robot
        current_distance = OBSTACLE_SENSOR.distance()
        if current_distance < STOP_DISTANCE:
            wait_for_obstacle_removal()
            continue 

        # Read current reflection from the color sensor
        current_reflection = LINE_SENSOR.reflection()
        
        # Count consecutive readings off the line to detect complete line loss
        if current_reflection > TARGET_THRESHOLD + LOST_LINE_TOLERANCE:
            lost_line_counter += 1
            if lost_line_counter > LOST_LINE_COUNT_MAX:
                mission_lost_count = search_for_line(mission_lost_count)
                lost_line_counter = 0
        else:
            lost_line_counter = 0

        # --- PD Controller Logic ---
        # 1. Proportional Error (deviation from the target edge)
        error = current_reflection - TARGET_THRESHOLD

        # 2. Derivative Error (rate of change of error to anticipate curves)
        derivative = error - last_error

        # 3. Compute steering turn rate correction
        turn_rate = (PROPORTIONAL_GAIN * error) + (DERIVATIVE_GAIN * derivative)

        # Clamp turn rate to safe maximum limits
        if turn_rate > MAX_TURN_RATE:
            turn_rate = MAX_TURN_RATE
        elif turn_rate < -MAX_TURN_RATE:
            turn_rate = -MAX_TURN_RATE

        # Apply movement command to DriveBase
        robot.drive(DRIVE_SPEED, turn_rate)

        # Store current error for the next loop's derivative calculation
        last_error = error

        # Small pause for loop stability
        wait(10)

if __name__ == "__main__":
    run_mission()