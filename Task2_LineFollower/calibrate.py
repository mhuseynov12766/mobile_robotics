#!/usr/bin/env pybricks-micropython

from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, ColorSensor
from pybricks.parameters import Port
from pybricks.robotics import DriveBase
from pybricks.tools import wait, StopWatch, DataLog

# Initialize EV3 brick and motors
ev3 = EV3Brick()
left_motor = Motor(Port.B)
right_motor = Motor(Port.C)

# Setup DriveBase and Color Sensor (adjust ports if necessary)
robot = DriveBase(left_motor, right_motor, wheel_diameter=55, axle_track=120)
sensor = ColorSensor(Port.S3)

# Initialize DataLog to save records into a CSV file on the brick
# (Similar to the DataLog usage specified in the lab report)[cite: 3, 6]
data_log = DataLog('time_ms', 'reflection')
timer = StopWatch()

def main():
    ev3.screen.clear()
    ev3.screen.print("Ready to Log!")
    ev3.screen.print("Press center button")
    ev3.screen.print("to start sweep.")
    
    # Wait for center button press before starting the run
    while not any(ev3.buttons.pressed()):
        wait(10)
    while any(ev3.buttons.pressed()):
        wait(10) # Debounce
        
    ev3.screen.clear()
    ev3.screen.print("Logging active...")
    ev3.speaker.beep()
    
    # Drive very slowly (20 mm/s) in a straight line across the floor and tape
    sweep_speed = 20 
    robot.drive(sweep_speed, 0)
    
    timer.reset()
    
    # Run the profiling sweep for 10 seconds (adjust time based on track length)
    while timer.time() < 10000:
        current_time = timer.time()
        reflection = sensor.reflection()
        
        # 1. Print live updates on the EV3 brick screen
        ev3.screen.clear()
        ev3.screen.print("Time: " + str(current_time) + " ms")
        ev3.screen.print("Reflection: " + str(reflection))
        
        # 2. Log data row-by-row to the CSV file on the brick[cite: 3, 6]
        data_log.log(current_time, reflection)
        
        # Sample every 50 ms (matching the lab's loop period)[cite: 3]
        wait(50)
        
    # Stop the robot and signal completion
    robot.stop()
    ev3.speaker.beep(1000, 200)
    
    ev3.screen.clear()
    ev3.screen.print("Logging Finished!")
    ev3.screen.print("File saved on brick.")
    
    # Hold screen active until any button is pressed
    while not any(ev3.buttons.pressed()):
        wait(100)

if __name__ == "__main__":
    main()