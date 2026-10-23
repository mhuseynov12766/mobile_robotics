#!/usr/bin/env pybricks-micropython

from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor
from pybricks.parameters import Port

# Initialize the EV3 Brick and motors
ev3 = EV3Brick()
left_motor = Motor(Port.A)
right_motor = Motor(Port.D)

# Drive both wheels forward for 5 rotations 
ev3.speaker.beep()

left_motor.run(150, 5 * 360, wait=False)  
right_motor.run(150, 5 * 360, wait=True)

ev3.speaker.beep()