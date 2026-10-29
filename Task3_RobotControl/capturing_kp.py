#!/usr/bin/env pybricks-micropython

from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, UltrasonicSensor
from pybricks.parameters import Port
from pybricks.robotics import DriveBase
from pybricks.tools import wait, StopWatch, DataLog

# Initialize EV3 brick and hardware ports
ev3 = EV3Brick()
left_motor = Motor(Port.B)
right_motor = Motor(Port.C)
eyes = UltrasonicSensor(Port.S4)

# Robot physical specs from Task 1 & 2 calibration
WHEEL_DIAMETER = 54.8
AXLE_TRACK = 109.6
robot = DriveBase(left_motor, right_motor, WHEEL_DIAMETER, AXLE_TRACK)

# --- Phase 1: Proportional Sweep Configuration ---
SETPOINT = 200        # Target distance from wall in mm
UMAX = 300            # Motor command clamp limit (mm/s)

# Change this value for each sweep test: 0.5, 1.0, 2.0, 4.0, 8.0
KP = 8.0              

# Setup DataLog for CSV output (t_ms, d_mm, e_mm, u_mms)
# Tip: Rename the log name per run (e.g., 'sweep_kp05', 'sweep_kp1', etc.)
log = DataLog('t_ms', 'd_mm', 'e_mm', 'u_mms', name='sweep_kp')
clock = StopWatch()

def run_proportional_sweep():
    ev3.speaker.beep()
    clock.reset()

    while True:
        d = eyes.distance()
        
        # Guard: Skip pass if ultrasonic sensor sees nothing (2550 mm)[cite: 38, 106, 122]
        if d > 2500:
            wait(50)
            continue
            
        # Compute error (e = d - SETPOINT)
        e = d - SETPOINT
        
        # Proportional control law only (u = Kp * e)
        u = KP * e
        
        # Apply command clamping (±300 mm/s)[cite: 40, 62]
        u_clamped = max(-UMAX, min(UMAX, u))
        
        # Drive robot straight (turn rate = 0)[cite: 40]
        robot.drive(u_clamped, 0)
        
        # Log row data to CSV
        log.log(clock.time(), d, e, u_clamped)
        
        wait(50)

if __name__ == '__main__':
    run_proportional_sweep()