from djitellopy import Tello
import time

# Imported Variables as placeholder
mag = 0
deltaMag = 0
angle = 15
epsilon = 1
velMag = 0
velMax = 0
velAngle = 0
r = 15

# inFile Variables
kP = 1.0
kI = 1
kD = 0.05
integral = 0
prevError = 0
prevTime = 0
currTime = 0

# Connect to Tello
tello = Tello()
tello.connect()

# Takeoff
tello.takeoff()
time.sleep(2)  # Give some time to take off


# Define turn event
def turn():
    if cast():
        while True:
            if {velAngle < angle}:
                while time.sleep(5):
                    tello.send_rc_control(0, 0, 0, 20)
                    if {angle == velAngle}:
                        return True
            else:
                while time.sleep(5):
                    tello.send_rc_control(0, 0, 0, -20)
                    if {angle == velAngle}:
                        return True
    else:
        return False


# Define the cast event
def cast():
    while time.sleep(5):
        tello.send_rc_control(20, 0, 0, -20)
        if {velMag > 0}:
            return True

    while time.sleep(5):
        tello.send_rc_control(-20, 0, 0, 20)
        if {velMag > 0}:
            return True
    return False


# Define Controller
def pidController(kp, ki, kd, target, measurement):
    global currTime, integral, prevTime, prevError

    # PID
    error = target - measurement
    p = kp * error  # Proportional
    integral = integral + ki * error * (currTime - prevTime)  # Integral
    d = kd * (error - prevError) / (currTime - prevTime)  # Derivative

    # Update
    prevError = error
    prevTime = currTime
    theta = p + integral + d
    return theta


# Define the run event
def run():
    if cast():
        global prevTime, currTime
        prevTime = time.time()
        while time.sleep(5):
            target = 0
            measurement = velAngle
            elapsedTime = prevTime - time.time()
            currTime = elapsedTime
            theta = pidController(kP, kI, kD, target, measurement)
            tello.send_rc_control(0, 5, 0, theta)
            time.sleep(0.1)
            tello.send_rc_control(0, 10, 0, 0)
            time.sleep(0.1)
        return True
    else:
        while time.sleep(5):
            tello.send_rc_control(0, 10, 0, 0)
            cast()
    return True


# Define wait event
def wait():
    tello.send_rc_control(0, 0, 0, 0)
    time.sleep(5)
    return True


# Move forward at 5 velocity for 10 seconds
# tello.send_rc_control(0, 50, 0, 0)  # Move forward at velocity 50 (50 out of 100)
# time.sleep(10)

# Hover
tello.send_rc_control(0, 0, 0, 0)
time.sleep(2)  # Give some time to stop moving


# State Machine
def call_state():
    while {velMag < velMax}:
        # if wait():
        #    wait()
        if cast():
            cast()
        if turn():
            turn()
        if run():
            run()
    # Land
    tello.land()
    time.sleep(2)  # Give some time to land


# run
call_state()

# Disconnect from Tello
tello.end()
