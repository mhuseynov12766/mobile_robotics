#!/usr/bin/env pybricks-micropython

from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, ColorSensor, UltrasonicSensor
from pybricks.parameters import Port
from pybricks.robotics import DriveBase
from pybricks.tools import StopWatch, wait
from random import choice, uniform


# Setup the EV3 Brick and devices

ev3 = EV3Brick()

left_motor = Motor(Port.B)
right_motor = Motor(Port.C)
eye = ColorSensor(Port.S3)
eyes = UltrasonicSensor(Port.S4)

robot = DriveBase(left_motor, right_motor, 54.8, 109.6)


# Setup the arena parameters

TAPE = 25
ROBOT_MM = 250
REVERSE_MM = 150

SPEED = 150
TURN_RATE = 120

LOOP_MS = 50
CONFIRM = 2
AVOID_MIN_S = 0.6


# Setup the robot's memory

st = {
    'escape': 'WANDER',
    'near_n': 0,
    'avoid_until': -1,
    'last_turn': 0,
    'wander_until': 0,
    'wander_turn': 0,
    'wander_mark': 0
}


# Setup the timer

watch = StopWatch()


# Define the escape behaviour

def escape(s, st):
    state = st['escape']

    if state == 'WANDER':
        if s['reflection'] < TAPE:
            st['escape'] = 'REVERSE'
            st['mark'] = s['odo_mm']
            return (-SPEED, 0)

        return None

    if state == 'REVERSE':
        if st['mark'] - s['odo_mm'] >= REVERSE_MM:
            st['escape'] = 'TURN'
            st['mark'] = s['odo_deg']
            st['turn'] = choice((-1, 1)) * uniform(90, 180)

            return (0, TURN_RATE if st['turn'] > 0
                    else -TURN_RATE)

        return (-SPEED, 0)

    if state == 'TURN':
        if abs(s['odo_deg'] - st['mark']) >= abs(st['turn']):
            st['escape'] = 'WANDER'
            st['last_turn'] = 1 if st['turn'] > 0 else -1
            return None

        return (0, TURN_RATE if st['turn'] > 0
                else -TURN_RATE)

    return None


# Define the collision avoidance behaviour

def avoid(s, st):
    near = s['distance'] < ROBOT_MM

    if near:
        st['near_n'] += 1
    else:
        st['near_n'] = 0

    active = st['avoid_until'] > s['t']

    if st['near_n'] >= CONFIRM and not active:
        st['side'] = st['last_turn'] or choice((-1, 1))
        st['avoid_until'] = s['t'] + AVOID_MIN_S
        active = True

    if active:
        return (0, st['side'] * TURN_RATE)

    return None


# Define the wandering behaviour

def wander(s, st):
    if st['wander_until'] > s['t']:
        if abs(s['odo_deg'] - st['wander_mark']) >= abs(st['wander_turn']):
            st['wander_until'] = 0
            return (SPEED, 0)

        return (0, TURN_RATE if st['wander_turn'] > 0
                else -TURN_RATE)

    if s['t'] >= st.get('next_change', 0):
        st['wander_mark'] = s['odo_deg']
        st['wander_turn'] = choice((-1, 1)) * uniform(30, 90)
        st['wander_until'] = s['t'] + 3
        st['next_change'] = s['t'] + uniform(3, 6)

        return (0, TURN_RATE if st['wander_turn'] > 0
                else -TURN_RATE)

    return (SPEED, 0)


# Define the referee to select the highest-priority behaviour

def referee(behaviours, s, st):
    for behaviour in behaviours:
        command = behaviour(s, st)

        if command is not None:
            return command

    return (0, 0)


# Set the behaviour priority

behaviours = [escape, avoid, wander]


# Start the robot and beep to indicate readiness

ev3.speaker.beep()
watch.reset()


# Run the arena program

while True:
    s = {
        't': watch.time() / 1000,
        'reflection': eye.reflection(),
        'distance': eyes.distance(),
        'odo_mm': robot.distance(),
        'odo_deg': robot.angle()
    }

    command = referee(behaviours, s, st)

    robot.drive(command[0], command[1])

    wait(LOOP_MS)