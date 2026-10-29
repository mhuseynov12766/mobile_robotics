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

# --- Phase 2: Controller Configuration ---
SETPOINT = 200        # Target distance from wall in mm
H = 0.05              # Loop period in seconds (50 ms)
UMAX = 300            # Motor command clamp limit (mm/s)
ALPHA = 0.3           # Derivative low-pass filter coefficient

# Choose your controller type: "P", "PD", or "PID"
CONTROLLER_TYPE = "PID"

# Final Tuned Gains (Adjust these based on your tests)
# PD recommended:  Kp = 8.0, Ki = 0.0, Kd = 0.6
# PID recommended: Kp = 6.0, Ki = 3.0, Kd = 0.5
KP = 6.0              # Proportional gain (1/s)
KI = 3.0              # Integral gain (1/s^2)
KD = 1.0              # Derivative gain (dimensionless)

# Setup DataLog for CSV output (t_ms, d_mm, e_mm, u_mms)
# Tip: Name the log according to your run, e.g., 'A_PID', 'B_PD', 'C_P', etc.
log = DataLog('t_ms', 'd_mm', 'e_mm', 'u_mms', name='run_log')
clock = StopWatch()

def run_controller():
    ev3.speaker.beep()
    
    integral = 0.0
    e_prev = 0.0
    deriv = 0.0
    
    clock.reset()

    while True:
        d = eyes.distance()
        
        # Guard: Skip pass if ultrasonic sensor sees nothing (2550 mm)[cite: 38, 106, 122]
        if d > 2500:
            wait(50)
            continue
            
        # Compute error (e = d - SETPOINT)
        e = d - SETPOINT
        
        # 1. Proportional Term
        p_term = KP * e
        
        # 2. Derivative Term with Low-Pass Filtering
        raw_deriv = (e - e_prev) / H
        deriv = (ALPHA * raw_deriv) + ((1 - ALPHA) * deriv)
        d_term = KD * deriv
        
        # 3. Compute raw control command u
        if CONTROLLER_TYPE == "P":
            u = p_term
        elif CONTROLLER_TYPE == "PD":
            u = p_term + d_term
        else: # PID
            u = p_term + (KI * integral) + d_term
            
        # Apply command clamping (±300 mm/s)[cite: 40, 62]
        u_clamped = max(-UMAX, min(UMAX, u))
        
        # 4. Anti-Windup: Freeze the integral while the command is clamped (PID only)[cite: 8, 78, 106]
        if CONTROLLER_TYPE == "PID":
            if abs(u) < UMAX:
                integral += e * H
                
        # Drive robot straight (turn rate = 0)[cite: 40]
        robot.drive(u_clamped, 0)
        
        # Log row data to CSV
        log.log(clock.time(), d, e, u_clamped)
        
        e_prev = e
        wait(50)

if __name__ == '__main__':
    run_controller()