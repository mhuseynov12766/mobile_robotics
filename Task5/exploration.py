#!/usr/bin/env pybricks-micropython

import os
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, ColorSensor, UltrasonicSensor
from pybricks.parameters import Port, Button
from pybricks.robotics import DriveBase
from pybricks.tools import wait, StopWatch

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

# Control parameters (Original PD parameters)
DRIVE_SPEED = 120       # Base forward speed in mm/s
PROPORTIONAL_GAIN = 1.8 # Kp: proportional gain for error correction
DERIVATIVE_GAIN = 2.0   # Kd: derivative gain to smooth turns
MAX_TURN_RATE = 110     # Maximum turn rate limit in deg/s

# Obstacle detection threshold (distance in mm)
STOP_DISTANCE = 125

# Line loss recovery parameters 
LOST_LINE_TOLERANCE = 5   
LOST_LINE_COUNT_MAX = 30  
SEARCH_TURN_RATE = 45     

RIGHT_SEARCH_LIMIT = 180  
LEFT_SEARCH_LIMIT = 360   

run_timer = StopWatch()

def get_next_log_filename():
    """Scans the EV3 filesystem to find the next available log name (log1.csv, log2.csv...)."""
    i = 1
    while True:
        filename = "log{}.csv".format(i)
        try:
            os.stat(filename)
            i += 1  
        except OSError:
            return filename  

def search_for_line(mission_lost_count):
    """Executes a sweep to recover the line if lost."""
    robot.stop()
    ev3.speaker.beep(100, 100)
    SEARCH_SPEED = 0
    
    initial_angle = LEFT_MOTOR.angle()
    while abs(LEFT_MOTOR.angle() - initial_angle) < RIGHT_SEARCH_LIMIT:
        robot.drive(SEARCH_SPEED, SEARCH_TURN_RATE)
        if LINE_SENSOR.reflection() <= (TARGET_THRESHOLD - LOST_LINE_TOLERANCE):
            robot.stop()
            ev3.speaker.beep(1000, 100)
            wait(200)
            return mission_lost_count + 1
        wait(10)

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
    robot.stop()
    ev3.speaker.beep(500, 100) 
    while OBSTACLE_SENSOR.distance() < STOP_DISTANCE:
        wait(200) 
    ev3.speaker.beep(1000, 100)
    wait(500)

def run_mission():
    ev3.speaker.beep()
    
    # Get auto-incrementing log filename (log1.csv, log2.csv, etc.)
    log_filename = get_next_log_filename()
    print("Creating log file:", log_filename)
    
    log_file = open(log_filename, "w")
    log_file.write("time_ms,distance_mm,reflection,turn_rate,state\n")
    
    lost_line_counter = 0
    mission_lost_count = 0
    last_error = 0
    
    current_state = "FOLLOW_OUTBOUND"
    
    robot.drive(DRIVE_SPEED, 0)
    run_timer.reset()
    robot.reset()

    try:
        while True:
            current_time = run_timer.time()
            current_distance = robot.distance()
            current_reflection = LINE_SENSOR.reflection()

            # --- MANUAL BUTTON PRESS TURNAROUND TRIGGER ---
            pressed_buttons = ev3.buttons.pressed()
            if pressed_buttons and current_state == "FOLLOW_OUTBOUND":
                ev3.speaker.beep(600, 150)
                print("Button pressed! Turning around to head back...")
                
                current_state = "TURNING_AROUND"
                log_line = "{},{},{},{},{}\n".format(current_time, current_distance, current_reflection, 0, current_state)
                log_file.write(log_line)

                # Stop and turn around by 180 degrees (or 360 if you want a full spin)
                robot.stop()
                robot.turn(180)
                
                print("Turnaround complete. Following line back to start.")
                current_state = "FOLLOW_INBOUND"
                robot.drive(DRIVE_SPEED, 0)

            # Obstacle handling
            if current_state in ["FOLLOW_OUTBOUND", "FOLLOW_INBOUND"] and OBSTACLE_SENSOR.distance() < STOP_DISTANCE:
                wait_for_obstacle_removal()
                continue 

            if current_state in ["FOLLOW_OUTBOUND", "FOLLOW_INBOUND"]:
                # Line loss check
                if current_reflection > (TARGET_THRESHOLD + LOST_LINE_TOLERANCE):
                    lost_line_counter += 1
                    if lost_line_counter > LOST_LINE_COUNT_MAX:
                        mission_lost_count = search_for_line(mission_lost_count)
                        lost_line_counter = 0
                else:
                    lost_line_counter = 0

                # --- PD Controller Logic ---
                error = current_reflection - TARGET_THRESHOLD
                derivative = error - last_error
                turn_rate = (PROPORTIONAL_GAIN * error) + (DERIVATIVE_GAIN * derivative)

                if turn_rate > MAX_TURN_RATE:
                    turn_rate = MAX_TURN_RATE
                elif turn_rate < -MAX_TURN_RATE:
                    turn_rate = -MAX_TURN_RATE

                robot.drive(DRIVE_SPEED, turn_rate)
                last_error = error

            # Log row to CSV
            log_line = "{},{},{},{},{}\n".format(current_time, current_distance, current_reflection, turn_rate, current_state)
            log_file.write(log_line)

            wait(10)

    finally:
        log_file.close()
        print("Log file saved successfully.")

if __name__ == "__main__":
    run_mission()