from djitellopy import Tello
import time
from math import *
import string

# Setup variables to be used in console simulation
velAngle = pi * int(input('Define incoming angle measured from center (Max: 45) \n')) / 180
velMag = int(input('Define magnitude of the source flow velocity \n'))
velMax = int(input('Define the maximum flow that the drone can detect \n'))
dimension = int(input('Define the maximum 2D space dimension \n'))

# Initialize vector space
space = []
for rows in range(dimension):
    space += [0]
    col = []
    for cols in range(dimension):
        col += [0]
    space[rows] = col

# Project flow onto vector space
xMax = int(dimension / 2)
slope = tan(velAngle)
sourcePosition = (xMax, slope * xMax)
for i in range(int(dimension / 2)):
    x = i
    y = int(slope * x)
    ratio = sqrt(x * x + y * y) / sqrt(sourcePosition[0] * sourcePosition[0] + sourcePosition[1] * sourcePosition[1])
    space[int(dimension / 2) - y][int(dimension / 2) + x] = ratio * velMag

# Initialize drone parameters
angle = 0
detectedVel = 0
dronePosition = (dimension, int(dimension / 2))

# inFile Variables
kP = 1.0
kI = 1
kD = 0.05
integral = 0
prevError = 0
prevTime = 0
currTime = 0

# Establish States
castState = bool
turnState = bool
runState = bool

# Connect to Tello
# tello = Tello()
# tello.connect()

# Start Time
begin = time.time()


# Takeoff
# tello.takeoff()
# time.sleep(2)  # Give some time to take off


# Define turn event
def turn():
    global velAngle, angle, castState
    # print('in turn')
    if castState:
        velAngle = int(round(velAngle * 180 / pi))
        if angle < velAngle:
            while angle != velAngle:
                # print(f"the flow is at an angle of {velAngle} degrees")
                # tello.send_rc_control(0, 0, 0, 20)
                angle += 1
                # print(f"the drone's angle is {angle} degrees")
                if angle == velAngle:
                    break
                time.sleep(0.5)
        else:
            while angle != velAngle:
                print(f'{velAngle}')
                # tello.send_rc_control(0, 0, 0, -20)
                angle -= 1
                elapsed = time.time() - lap1
            print(angle)
    return True


# Define the cast event
def cast():
    global dronePosition, detectedVel
    # print('in cast')
    lap1 = time.time()
    elapsed = 0
    while elapsed < 2:
        # tello.send_rc_control(20, 0, 0, -20)
        dronePosition = (dronePosition[0] - 1, dronePosition[1] - 1)
        detectedVel = space[dronePosition[1]][dronePosition[0]]
        print(dronePosition)
        elapsed = time.time() - lap1
        if detectedVel > 0:
            print('velocity detected')
            print(f'the detected velocity magnitude is {detectedVel}')
            return True
        time.sleep(0.5)
    print(f'{elapsed} seconds has passed since the loop started')
    elapsed = 0
    lap1 = time.time()
    while elapsed < 2:
        # tello.send_rc_control(-20, 0, 0, 20)
        dronePosition = (dronePosition[0] - 1, dronePosition[1] + 1)
        detectedVel = space[dronePosition[1]][dronePosition[0]]
        print(dronePosition)
        elapsed = time.time() - lap1
        if detectedVel > 0:
            print('velocity detected')
            print(f'the detected velocity magnitude is {detectedVel}')
            return True
        time.sleep(0.5)
    print(f'{elapsed} seconds has passed since the loop started')
    elapsed = 0
    return False


# Define Controller
def pidController(kp, ki, kd, target, measurement):
    global currTime, integral, prevTime, prevError

    # PID
    error = target - measurement
    p = kp * error  # Proportional
    currTime = time.time()
    integral = integral + ki * error * (currTime - prevTime)  # Integral
    d = kd * (error - prevError) / (currTime - prevTime)  # Derivative

    # Update
    prevError = error
    prevTime = currTime
    theta = p + integral + d
    return theta


# Define the run event
def run():
    global prevTime, dronePosition, detectedVel
    # print('in run')
    if castState:
        lap1 = time.time()
        while detectedVel < velMax:
            target = velAngle
            measurement = angle
            elapsedTime = time.time() - lap1
            prevTime = time.time()
            time.sleep(0.5)
            theta = pidController(kP, kI, kD, target, measurement)
            # tello.send_rc_control(0, 5, 0, theta)
            # time.sleep(0.1)
            # tello.send_rc_control(0, 10, 0, 0)
            # time.sleep(0.1)
            preVel = space[dronePosition[1]][dronePosition[0]]
            if preVel < space[dronePosition[1]][dronePosition[0] + 1]:
                detectedVel = space[dronePosition[1]][dronePosition[0] + 1]
                dronePosition = (dronePosition[0] + 1, dronePosition[1])
            elif preVel < space[dronePosition[1] - 1][dronePosition[0]]:
                detectedVel = space[dronePosition[1] - 1][dronePosition[0]]
                dronePosition = (dronePosition[0], dronePosition[1] - 1)
            else:
                detectedVel = space[dronePosition[1] - 1][dronePosition[0] + 1]
                dronePosition = (dronePosition[0] + 1, dronePosition[1] - 1)
        return True
    else:
        cast()
        return True


# Define wait event
def wait():
    # tello.send_rc_control(0, 0, 0, 0)
    time.sleep(5)
    return True


# Move forward at 5 velocity for 10 seconds
# tello.send_rc_control(0, 50, 0, 0)  # Move forward at velocity 50 (50 out of 100)
# time.sleep(10)

# Hover
# tello.send_rc_control(0, 0, 0, 0)
# time.sleep(2)  # Give some time to stop moving


# State Machine
def call_state():
    global detectedVel, velMax, castState, turnState, runState
    # print('inside function')
    while detectedVel < velMax:
        # print(detectedVel)
        # print(velMax)
        # if wait():
        #    wait()
        if cast():
            castState = True
            # print(f'if statement 1...{castState}')
        if turn():
            turnState = True
            # print(f'if statement 2...{turnState}')
        if run():
            runState = True
            # print(f'if statement 3...{runState}')
    # Land
    # tello.land()
    # time.sleep(2)  # Give some time to land


# run
call_state()
end = time.time() - begin
print(f'Drone ended with position ({dronePosition[0]}, {dronePosition[1]}) with a detected '
      f'velocity of {space[dronePosition[1]][dronePosition[0]]}m/s after {end} seconds!')
# Disconnect from Tello
# tello.end()
