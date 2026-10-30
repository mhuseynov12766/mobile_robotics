#!/usr/bin/env pybricks-micropython

import os
import heapq
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

# Control parameters (Stable cruise speed)
DRIVE_SPEED = 90        
PROPORTIONAL_GAIN = 1.8 
DERIVATIVE_GAIN = 2.0   
MAX_TURN_RATE = 110     

# Obstacle detection threshold (distance in mm)
STOP_DISTANCE = 125

# Line loss recovery parameters 
LOST_LINE_TOLERANCE = 5   
LOST_LINE_COUNT_MAX = 30  
SEARCH_TURN_RATE = 45     

RIGHT_SEARCH_LIMIT = 180  
LEFT_SEARCH_LIMIT = 360   

# Node Detection Thresholds
REF_JUNCTION_BLACK = 15.0  
JUNCTION_TRAVEL_MM = 30.0  

run_timer = StopWatch()

# --- Arena Graph Map (Weights derived from your CSV logs in mm) ---
ARENA_GRAPH = {
    'S': {'C': 1401, 'D4': 610, 'M': 572},
    'C': {'S': 1401, 'N': 1000, 'D1': 450},
    'N': {'C': 1000, 'K': 800, 'M': 1200, 'G2': 1314},
    'K': {'N': 800, 'G1': 1500, 'D2': 555},
    'M': {'S': 572, 'N': 1200, 'G2': 1200},
    'D1': {'C': 450},
    'D2': {'K': 555},
    'D4': {'S': 610},
    'G1': {'K': 1500},
    'G2': {'N': 1314, 'M': 1200}
}

def dijkstra(graph, start, goal):
    """Calculates the shortest path and total distance between start and goal nodes."""
    queue = [(0, start, [start])]
    visited = set()
    
    while queue:
        (cost, current, path) = heapq.heappop(queue)
        if current in visited:
            continue
        visited.add(current)
        
        if current == goal:
            return cost, path
            
        for next_node, weight in graph.get(current, {}).items():
            if next_node not in visited:
                heapq.heappush(queue, (cost + weight, next_node, path + [next_node]))
                
    return float("inf"), []

def get_next_log_filename():
    """Scans EV3 storage to find the next available log name (log1.csv, log2.csv...)."""
    i = 1
    while True:
        filename = "log{}.csv".format(i)
        try:
            os.stat(filename)
            i += 1  
        except OSError:
            return filename  

def search_for_line(mission_lost_count):
    """Robust line recovery sweep."""
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
    
    # Define route
    start_node = 'S'
    goal_node = 'G1'
    cost, path = dijkstra(ARENA_GRAPH, start_node, goal_node)
    print("Dijkstra Route:", path)
    
    # Initialize auto-incrementing log file
    log_filename = get_next_log_filename()
    print("Logging to:", log_filename)
    log_file = open(log_filename, "w")
    log_file.write("time_ms,distance_mm,reflection,turn_rate,state\n")
    
    lost_line_counter = 0
    mission_lost_count = 0
    last_error = 0
    
    black_distance_start = None
    current_state = "FOLLOW"
    
    robot.drive(DRIVE_SPEED, 0)
    run_timer.reset()
    robot.reset()

    try:
        while True:
            current_time = run_timer.time()
            current_distance = robot.distance()
            current_reflection = LINE_SENSOR.reflection()

            # Center button stops the program cleanly
            if Button.CENTER in ev3.buttons.pressed():
                ev3.speaker.beep(400, 200)
                robot.stop()
                print("Program terminated by user.")
                break

            # Obstacle check
            if current_state == "FOLLOW" and OBSTACLE_SENSOR.distance() < STOP_DISTANCE:
                wait_for_obstacle_removal()
                continue 

            if current_state == "FOLLOW":
                # Line loss tracking
                if current_reflection > (TARGET_THRESHOLD + LOST_LINE_TOLERANCE):
                    lost_line_counter += 1
                    if lost_line_counter > LOST_LINE_COUNT_MAX:
                        mission_lost_count = search_for_line(mission_lost_count)
                        lost_line_counter = 0
                else:
                    lost_line_counter = 0

                # --- Junction Detection ---
                if current_reflection < REF_JUNCTION_BLACK:
                    if black_distance_start is None:
                        black_distance_start = current_distance
                    elif (current_distance - black_distance_start) >= JUNCTION_TRAVEL_MM:
                        ev3.speaker.beep(800, 250)
                        robot.stop()
                        print("Junction Reached at distance: {} mm".format(current_distance))
                        current_state = "NODE_PAUSE"
                else:
                    black_distance_start = None

                # --- PD Controller ---
                error = current_reflection - TARGET_THRESHOLD
                derivative = error - last_error
                turn_rate = (PROPORTIONAL_GAIN * error) + (DERIVATIVE_GAIN * derivative)

                if turn_rate > MAX_TURN_RATE:
                    turn_rate = MAX_TURN_RATE
                elif turn_rate < -MAX_TURN_RATE:
                    turn_rate = -MAX_TURN_RATE

                robot.drive(DRIVE_SPEED, turn_rate)
                last_error = error

            elif current_state == "NODE_PAUSE":
                turn_rate = 0
                # Robot waits at the node. Press LEFT to turn left (-90), RIGHT to turn right (+90), or UP to go straight.
                pressed = ev3.buttons.pressed()
                if Button.LEFT in pressed:
                    ev3.speaker.beep(1000, 100)
                    robot.turn(-90)
                    robot.reset()
                    black_distance_start = None
                    current_state = "FOLLOW"
                    robot.drive(DRIVE_SPEED, 0)
                elif Button.RIGHT in pressed:
                    ev3.speaker.beep(1000, 100)
                    robot.turn(90)
                    robot.reset()
                    black_distance_start = None
                    current_state = "FOLLOW"
                    robot.drive(DRIVE_SPEED, 0)
                elif Button.UP in pressed:
                    ev3.speaker.beep(1000, 100)
                    robot.straight(50)  # Creep past junction line
                    robot.reset()
                    black_distance_start = None
                    current_state = "FOLLOW"
                    robot.drive(DRIVE_SPEED, 0)

            # Log row to CSV
            log_line = "{},{},{},{},{}\n".format(current_time, current_distance, current_reflection, turn_rate, current_state)
            log_file.write(log_line)

            wait(10)

    finally:
        log_file.close()
        print("Log saved successfully.")

if __name__ == "__main__":
    run_mission()