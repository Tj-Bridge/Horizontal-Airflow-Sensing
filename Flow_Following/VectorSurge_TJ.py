from djitellopy import Tello
import time
from math import *
import string
import asyncio
from bleak import BleakClient
import time
import csv
import keyboard
import os
import math
from djitellopy import Tello
import numpy as np

# Initialize drone parameters
angle = 0
sensorMagnitude = 0

# inFile Variables
Kp = 0.85
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

desiredAngle = 90
angle_threshold = 5

y_lowerBound = -10
y_upperBound = 10
magThreshold = 2.5

avg_x = 0
avg_y = 0


# Define turn event
def turn():
    global avg_x, avg_y, velAngle, angle, castState, sensorMagnitude, sourceAngle
    # print('in turn')
    if castState:
        sourceAngle = (int(np.arctan2(avg_y, avg_x) * (180 / np.pi))) % 360
        error = desiredAngle - sourceAngle
        forward_command = -20
        # delta_time = current_time - last_time
        #
        # integral += error * delta_time
        # derivative = (error - last_error) / delta_time if last_error is not None else 0
        #
        command = int(Kp * error)  # + Ki * integral + Kd * derivative)
        #
        last_error = error
        # last_time = current_time
        # if abs(error) < 45:
        #     Kp = 1.5
        if abs(error) <= angle_threshold:  # if error is within desired threshold, hover
            print("Within Threshold")
            tello.send_rc_control(0,forward_command,0,0)
            command = 0
            time.sleep(0.01)
            # if error is in Q1 or Q4 rotate right
            # updating logic to use source angle in if statemetnt condition
        elif 0 <= sourceAngle <= (desiredAngle - angle_threshold) or -270 <= sourceAngle <= -180:
            right_command = command
            right_command = max(min(right_command, 100), -100)
            print("Turning right, source angle: " + str(sourceAngle))
            tello.send_rc_control(0, forward_command, 0, right_command)  # velocity-based
            time.sleep(0.01)

        # if error is in Q2 or Q3 rotate left
        else:
            left_command = -command
            left_command = max(min(left_command, 100), -100)
            print("Turning left, source angle: " + str(sourceAngle))
            tello.send_rc_control(0, forward_command, 0, left_command)  # velocity-based
            time.sleep(0.01)
        return True
    else:
        return False

# Define the cast event
def cast():
    global avg_x, avg_y, sensorMagnitude, magThreshold
    lap1 = time.time()
    initial_cast_time = 3  # Starting cast time
    increment_time = 1  # Time increment per loop
    forward_move_time = 0.5  # Time to move forward (in seconds)

    sensorMagnitude = math.sqrt(avg_x ** 2 + avg_y ** 2)
    # Update the goal cast time for the left movement
    goalCastTime = initial_cast_time + increment_time * (time.time() - lap1)

    # Reset the elapsed time for this cast
    elapsed = 0
    if sensorMagnitude > magThreshold:
        sensorMagnitude = math.sqrt(avg_x ** 2 + avg_y ** 2)
        print('Flow detected, following')
        return True
    # Move Left

    lap1 = time.time()
    while elapsed < goalCastTime:
        tello.send_rc_control(-10, 0, 0, 0)  # Move left
        elapsed = time.time() - lap1
        sensorMagnitude = math.sqrt(avg_x ** 2 + avg_y ** 2)
        print("Moving left, Magnitude: " + str(sensorMagnitude))
        if sensorMagnitude > magThreshold:
            print('Flow detected, following')
            return True


        time.sleep(0.1)  # Small delay to prevent too frequent checks

    # Move Forward
    print('Moving forward...')
    elapsed = 0
    lap1 = time.time()
    while elapsed < forward_move_time:
        tello.send_rc_control(0, 20, 0, 0)  # Move forward
        elapsed = time.time() - lap1
        sensorMagnitude = math.sqrt(avg_x ** 2 + avg_y ** 2)
        print("Moving forward, Magnitude: " + str(sensorMagnitude))
        if sensorMagnitude > magThreshold:
            print('Flow detected, following')
            return True

        time.sleep(0.1)

    # Move Right
    print('Moving right...')
    elapsed = 0
    lap1 = time.time()
    while elapsed < goalCastTime:
        tello.send_rc_control(10, 0, 0, 0)  # Move right
        elapsed = time.time() - lap1
        sensorMagnitude = math.sqrt(avg_x ** 2 + avg_y ** 2)
        print("Moving right, Magnitude: " + str(sensorMagnitude))
        if sensorMagnitude > magThreshold:
            print('Flow detected, following')
            return True

        time.sleep(0.1)
    # Move Forward
    print('Moving forward...')
    elapsed = 0
    lap1 = time.time()
    while elapsed < forward_move_time:
        tello.send_rc_control(0, 20, 0, 0)  # Move forward
        elapsed = time.time() - lap1
        sensorMagnitude = math.sqrt(avg_x ** 2 + avg_y ** 2)
        print("Moving forward, Magnitude: " + str(sensorMagnitude))
        if sensorMagnitude > magThreshold:
            print('Flow detected, following')
            return True

        time.sleep(0.1)

    # Increment cast time for the next loop
    initial_cast_time += increment_time
    return False

#
# # Define the run event
# def run():
#     global prevTime, dronePosition, sensorMagnitude
#     # print('in run')
#     if castState:
#         lap1 = time.time()
#         while sensorMagnitude < 18:
#             target = velAngle
#             measurement = angle
#             elapsedTime = time.time() - lap1
#             prevTime = time.time()
#             time.sleep(0.1)
#             yaw_rate = pidController(kP, kI, kD, target, measurement)
#             tello.send_rc_control(0, 30, 0, yaw_rate)
#             time.sleep(0.1)
#         tello.land()
#         return True
#     else:
#         cast()
#         return True



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


# # State Machine
# def call_state():
#     global sensorMagnitude, thresholdMag, castState, turnState, runState
#
#     while sensorMagnitude < 100:
#         print("sensor Mag = " + str(sensorMagnitude))
#         if cast():
#             castState = True
#         if turn():
#             turnState = True
        # if run():
        #     runState = True

# run
async def main():
    global avg_x, avg_y, tello, sourceAngle, sensorMagnitude,castState, turnState, runState
    # BLE setup
    address = "18:2D:E3:60:9B:30"  # Change for your specific device
    characteristic_uuid = "00002A56-0000-1000-8000-00805F9B34FB"

    # Create a BleakClient object
    client = BleakClient(address)

    # Connect to the peripheral
    await client.connect()
    start_time = time.time()

    tello = Tello()
    tello.connect()
    print(tello.query_battery())

    # Makes drone takeoff
    tello.takeoff()
    time.sleep(2)
    tello.move_up(20)

    # Initialize calibration and filtering
    x_values = []
    y_values = []
    calibration_done = False
    x_offset = 0.0
    y_offset = 0.0
    x_buffer = []
    y_buffer = []
    calMag = []
    left = 0
    right = 0
    num_points_for_averaging = 10
    calibration_values_count = 10
    maxCalMag = []
    desiredAngle = 90

    last_magnitude = None
    while True:
        data = await client.read_gatt_char(characteristic_uuid)
        if data:
            string_n = data.decode()
            ard_string = string_n.strip()
            ard_list = ard_string.split()
            x = float(ard_list[1])
            y = float(ard_list[2])
            elapsed_time = time.time() - start_time
            # calibrate once
            # make sampling a function 
            if not calibration_done:
                if len(x_values) < calibration_values_count:
                    x_values.append(x)
                    y_values.append(y)
                    if len(x_values) == calibration_values_count:
                        x_offset = sum(x_values) / calibration_values_count
                        y_offset = sum(y_values) / calibration_values_count
                        calMag = [math.sqrt((xi - x_offset) ** 2 + (yi - y_offset) ** 2) for xi, yi in
                                  zip(x_values, y_values)]
                        maxCalMag = max(calMag)
                        calibration_done = True

                continue

            # Calibration adjustment
            x -= x_offset
            y -= y_offset

            # Buffer update for x and y
            x_buffer.append(x)
            y_buffer.append(y)

            # Averaging Filter
            if len(x_buffer) > num_points_for_averaging:
                x_buffer.pop(0)
            if len(y_buffer) > num_points_for_averaging:
                y_buffer.pop(0)
            if len(x_buffer) == num_points_for_averaging and len(y_buffer) == num_points_for_averaging:
                avg_x = sum(x_buffer) / num_points_for_averaging
                avg_y = sum(y_buffer) / num_points_for_averaging

                # calibration
                avg_x = avg_x * 0.7536477847
                avg_y = avg_y * 1.272200653

                # core variables


                # if magnitude is greater than magnitude while drone is hovering without the fan on
                if cast():
                    castState = True
                if turn():
                    turnState = True
                # print(f"Average Sensor Magnitude: {sensor_magnitude}, Time: {elapsed_time}, Angle: {angle}")

        if keyboard.is_pressed('space'):
            print("Spacebar pressed. Exiting.")
            tello.query_battery()
            tello.land()
            break


    await client.disconnect()
asyncio.run(main())

# Disconnect from Tello
# tello.end()
