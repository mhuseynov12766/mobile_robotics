#!/usr/bin/env pybricks-micropython

from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, ColorSensor, UltrasonicSensor
from pybricks.parameters import Port
from pybricks.robotics import DriveBase
from pybricks.tools import wait, StopWatch, DataLog

# Initialize the EV3 brick
ev3 = EV3Brick()

# Initialize drive motors (Port B for left, Port C for right)
LEFT_MOTOR = Motor(Port.B)
RIGHT_MOTOR = Motor(Port.C)

# Initialize sensors (Color sensor on Port 3, Ultrasonic on Port 4)
LINE_SENSOR = ColorSensor(Port.S3)
OBSTACLE_SENSOR = UltrasonicSensor(Port.S4)

# Setup robot dimensions for the DriveBase using your precise measurements
WHEEL_DIAMETER = 54.8
AXLE_TRACK = 109.6
robot = DriveBase(LEFT_MOTOR, RIGHT_MOTOR, WHEEL_DIAMETER, AXLE_TRACK)

# Calibration values measured from your lab floor (62) and tape (~7.5)
BLACK_REFLECTION = 7.5
WHITE_REFLECTION = 62.0

# Calculate the target threshold (the edge between floor and tape) -> ~34.75
TARGET_THRESHOLD = (BLACK_REFLECTION + WHITE_REFLECTION) / 2

# Control parameters (slower speed for stability)
DRIVE_SPEED = 120       # Base forward speed in mm/s
PROPORTIONAL_GAIN = 1.8 # Kp: proportional gain for error correction
DERIVATIVE_GAIN = 2.0   # Kd: derivative gain to smooth turns
MAX_TURN_RATE = 110     # Maximum turn rate limit in deg/s

# Obstacle detection threshold (distance in mm)
STOP_DISTANCE = 125

# Line loss recovery parameters 
LOST_LINE_TOLERANCE = 5   # Margin around target to confirm line detection
LOST_LINE_COUNT_MAX = 30  # Consecutive loops off track before triggering search
SEARCH_TURN_RATE = 45     # Controlled turn speed while searching

# Motor angle limits for thorough left and right scanning sweeps
RIGHT_SEARCH_LIMIT = 180  # Degrees to turn right
LEFT_SEARCH_LIMIT = 360   # Total degrees to sweep left across center

# Initialize DataLog to record run evidence onto the EV3 brick storage
run_logger = DataLog('time_ms', 'reflection', 'turn_rate')
run_timer = StopWatch()

def search_for_line(mission_lost_count):
    """
    Executes an expanded, thorough motor-encoder-based sweep: 
    First turns right deeply, then sweeps wide to the left to capture the line.
    """
    robot.stop()
    ev3.speaker.beep(100, 100)
    
    SEARCH_SPEED = 0
    
    # --- STEP 1: Deep Sweep Right ---
    initial_angle = LEFT_MOTOR.angle()
    while abs(LEFT_MOTOR.angle() - initial_angle) < RIGHT_SEARCH_LIMIT:
        robot.drive(SEARCH_SPEED, SEARCH_TURN_RATE)
        if LINE_SENSOR.reflection() <= (TARGET_THRESHOLD - LOST_LINE_TOLERANCE):
            robot.stop()
            ev3.speaker.beep(1000, 100)
            wait(200)
            return mission_lost_count + 1
        wait(10)

    # --- STEP 2: Wide Sweep Left across center and past it ---
    initial_angle = LEFT_MOTOR.angle()
    while abs(LEFT_MOTOR.angle() - initial_angle) < LEFT_SEARCH_LIMIT:
        robot.drive(SEARCH_SPEED, -SEARCH_TURN_RATE)
        if LINE_SENSOR.reflection() <= (TARGET_THRESHOLD - LOST_LINE_TOLERANCE):
            robot.stop()
            ev3.speaker.beep(1000, 100)
            wait(200)
            return mission_lost_count + 1
        wait(10)

    return mission_lost_count + 1
    
def wait_for_obstacle_removal():
    """Pauses the robot when an obstacle is detected until it is cleared."""
    robot.stop()
    ev3.speaker.beep(500, 100) 
    
    while OBSTACLE_SENSOR.distance() < STOP_DISTANCE:
        wait(200) 
    
    ev3.speaker.beep(1000, 100)
    wait(500)

def run_mission():
    """Main execution loop for PD line following, obstacle monitoring, and data logging."""
    ev3.speaker.beep()
    
    lost_line_counter = 0
    mission_lost_count = 0
    last_error = 0
    
    robot.drive(DRIVE_SPEED, 0)
    run_timer.reset()

    while True:
        # Check distance to any oncoming obstacle or robot
        current_distance = OBSTACLE_SENSOR.distance()
        if current_distance < STOP_DISTANCE:
            wait_for_obstacle_removal()
            continue 

        # Read current reflection from the color sensor
        current_reflection = LINE_SENSOR.reflection()

        # Count consecutive readings off the line to detect complete line loss
        if current_reflection > (TARGET_THRESHOLD + LOST_LINE_TOLERANCE):
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

        # Record operational data row-by-row into the CSV log file on the brick
        run_logger.log(run_timer.time(), current_reflection, turn_rate)

        # Store current error for the next loop's derivative calculation
        last_error = error

        wait(10)

if __name__ == "__main__":
    run_mission()