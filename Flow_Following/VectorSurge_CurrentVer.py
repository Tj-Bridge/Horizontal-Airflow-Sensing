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
import pandas
import matplotlib.pyplot as plt
import numpy as np
from numpy.f2py.crackfortran import endifs

# Initialize drone/sensor parameters
angle = 0
sensorMagnitude = 0
castSpeed = 30#10#20

# Initialize calibration and filtering
x_values = []
y_values = []
calibration_done = False
castCalibrated = False
x_offset = 0.0
y_offset = 0.0
x_buffer = []
y_buffer = []
calMag = []
left = 0
right = 0
num_points_for_averaging = 10
calibration_values_count = 15
maxCalMag = []
ambientMag = 0
magTolerance = 1.3
desiredAngle = 90

# Backtracking Variables
last_error = 0

# PID Controller Variables
Kp = 0.6
Ki = 0
Kd = 0.08
integral = 0
prevError = 0
prevTime = 0
currTime = 0

# Establish States
castState = bool
turnState = bool
runState = bool

angle_threshold = 5
sourceAngle = 0

y_lowerBound = -10
y_upperBound = 10
magThreshold = 1.2
magLimit = 6.5

#Cast Variables
initial_cast_time = 4
increment_time = 2
forward_move_time = 0.5

# BLE setup
address = "18:2D:E3:60:9B:30"  # Change for your specific device
characteristic_uuid = "00002A56-0000-1000-8000-00805F9B34FB"

# Create a BleakClient object
client = BleakClient(address)

# Create Tello drone object
tello = Tello()

def get_unique_filename(base=r"C:\Users\ltjth\Documents\Research\VectorSurge_30", extension=".csv"):
    counter = 1
    while os.path.exists(f"{base}_{counter}{extension}"):
        counter += 1
    return f"{base}_{counter}{extension}"

filename = get_unique_filename()
csv_file = open(filename, mode='w', newline='')
csv_writer = csv.writer(csv_file)
csv_writer.writerow(['Time', 'Bx', 'By','Sensor Angle', 'Sensor Magnitude', 'Command', 'Error', 'State'])

# Define turn event
async def turn():
    global angle, castState, sensorMagnitude, sourceAngle, magLimit, last_error, Ki, Kd, Kp, integral, prevTime
    # print('in turn')
    if castState:
        while sensorMagnitude >= magThreshold:
            avg_x, avg_y = await get_xy()
            sourceAngle = (int(np.arctan2(avg_y, avg_x) * (180 / np.pi)))+180 % 360
            sensorMagnitude = math.sqrt(avg_x ** 2 + avg_y ** 2) / ambientMag
            error = desiredAngle - sourceAngle
            forward_command = 60
            current_time = time.time()
            if prevTime is None or prevTime == 0:
                prevTime = current_time - 0.1
            error = desiredAngle - sourceAngle
            delta_time = current_time - prevTime
            integral += error * delta_time
            # derivative = (error - last_error) / delta_time #if last_error is not None else 0
            #
            command = int(Kp * error + Ki * integral)  # + Kd * derivative)
            #
            last_error = error
            prevTime = current_time

            if abs(error) <= angle_threshold:  # if error is within desired threshold, hover
                print("Within Threshold. Moving towards detected flow at " + str(
                    sourceAngle) + ' degrees with a magnitude of ' + str(sensorMagnitude))
                tello.send_rc_control(0, forward_command, 0, 0)
                sensorMagnitude = math.sqrt(avg_x ** 2 + avg_y ** 2) / ambientMag
                csv_writer.writerow(
                    [time.strftime("%Y-%m-%d %H:%M:%S"), avg_x,avg_y, sourceAngle, sensorMagnitude, 0, error, 2])
                time.sleep(0.1)

                # if error is in Q1 or Q4 rotate right
                # updating logic to use source angle in if statement condition
                # Changing the line below to reflect positive values. If turning does not work, switch back
            elif (270 <= sourceAngle <= 360) or (0 <= sourceAngle <= (desiredAngle - angle_threshold)):
                right_command = abs(command)
                right_command = max(min(right_command, 100), 0)
                print(right_command)
                print("Turning right, source angle & Magnitude: " + str(sourceAngle) + ':' + str(
                    sensorMagnitude) + " error: " + str(error))
                tello.send_rc_control(0, 0, 0, right_command)  # velocity-based
                sensorMagnitude = math.sqrt(avg_x ** 2 + avg_y ** 2) / ambientMag
                csv_writer.writerow(
                    [time.strftime("%Y-%m-%d %H:%M:%S"), avg_x, avg_y, sourceAngle, sensorMagnitude, right_command, error, 1])
                time.sleep(0.05)

            # if error is in Q2 or Q3 rotate left
            # Changing the line below to negate above elif statement. Change back to else statement if logic fails
            elif (desiredAngle + angle_threshold) <= sourceAngle < 270:
                left_command = command
                left_command = min(max(left_command, -100), 0)
                print(left_command)
                print("Turning left, source angle & Magnitude: " + str(sourceAngle) + ':' + str(
                    sensorMagnitude) + " error: " + str(error))
                tello.send_rc_control(0, 0, 0, left_command)  # velocity-based
                sensorMagnitude = math.sqrt(avg_x ** 2 + avg_y ** 2) / ambientMag
                csv_writer.writerow(
                    [time.strftime("%Y-%m-%d %H:%M:%S"), avg_x, avg_y, sourceAngle, sensorMagnitude, left_command, error, 1])
                time.sleep(0.05)
            last_error = error
            print(last_error)
            # End Condition for Flow Detection
            if sensorMagnitude > magLimit and abs(error) <= angle_threshold:
                csv_writer.writerow(
                    [time.strftime("%Y-%m-%d %H:%M:%S"), avg_x, avg_y, sourceAngle, sensorMagnitude, 0, error, 4])
                print("Maximum magnitude exceeded ... ending run")
                tello.land()
                tello.query_battery()
                await client.disconnect()
                break
    else:
        return False
    return True


# Define the cast event
async def cast():
    global sensorMagnitude, magThreshold, castCalibrated, ambientMag, magTolerance, initial_cast_time, increment_time, forward_move_time
    # Initialize Variable as list before calibrating
    if not castCalibrated:
        ambientMag = []

    lap1 = time.time()
    avg_x, avg_y = await get_xy()
    # sensorMagnitude = math.sqrt(avg_x ** 2 + avg_y ** 2) - ambientMag
    # Update the goal cast time for the left movement
    goalCastTime = initial_cast_time + increment_time * (time.time() - lap1)

    # Reset the elapsed time for this cast
    elapsed = 0
    if castCalibrated:
        if sensorMagnitude > magThreshold:
            avg_x, avg_y = await get_xy()
            sensorMagnitude = math.sqrt(avg_x ** 2 + avg_y ** 2) / ambientMag
            csv_writer.writerow(
                [time.strftime("%Y-%m-%d %H:%M:%S"), avg_x, avg_y, sourceAngle, sensorMagnitude, 0, 0, 0])
            print('Flow detected, following')
            return True

    # Move Left
    lap1 = time.time()
    while elapsed < goalCastTime/2:
        tello.send_rc_control(-castSpeed, 0, 0, 0)  # Move left
        elapsed = time.time() - lap1
        avg_x, avg_y = await get_xy()
        if castCalibrated:
            sensorMagnitude = math.sqrt(avg_x ** 2 + avg_y ** 2) / ambientMag
            csv_writer.writerow(
                [time.strftime("%Y-%m-%d %H:%M:%S"), avg_x, avg_y, sourceAngle, sensorMagnitude, 0, 0, 0])
        else:
            sensorMagnitude = math.sqrt(avg_x ** 2 + avg_y ** 2)
            csv_writer.writerow(
                [time.strftime("%Y-%m-%d %H:%M:%S"), avg_x, avg_y, sourceAngle, sensorMagnitude, 0, 0, 0])
            ambientMag.append(sensorMagnitude)
        print("Moving left, Magnitude: " + str(sensorMagnitude))
        if castCalibrated:
            if sensorMagnitude > magThreshold:
                print('Flow detected, following')
                return True
        time.sleep(0.1)  # Small delay to prevent too frequent checks

    # Move Forward
    print('Moving forward...')
    elapsed = 0
    lap1 = time.time()
    while elapsed < forward_move_time:
        tello.send_rc_control(0, castSpeed, 0, 0)  # Move forward
        elapsed = time.time() - lap1
        avg_x, avg_y = await get_xy()
        if castCalibrated:
            sensorMagnitude = math.sqrt(avg_x ** 2 + avg_y ** 2) / ambientMag
            csv_writer.writerow(
                [time.strftime("%Y-%m-%d %H:%M:%S"), avg_x, avg_y, sourceAngle, sensorMagnitude, 0, 0, 0])
        else:
            sensorMagnitude = math.sqrt(avg_x ** 2 + avg_y ** 2)
            csv_writer.writerow(
                [time.strftime("%Y-%m-%d %H:%M:%S"), avg_x, avg_y, sourceAngle, sensorMagnitude, 0, 0, 0])
            ambientMag.append(sensorMagnitude)
        print("Moving forward, Magnitude: " + str(sensorMagnitude))
        if castCalibrated:
            if sensorMagnitude > magThreshold:
                print('Flow detected, following')
                return True
        time.sleep(0.1)

    # Move Right
    print('Moving right...')
    elapsed = 0
    lap1 = time.time()
    while elapsed < goalCastTime:
        tello.send_rc_control(castSpeed, 0, 0, 0)  # Move right
        elapsed = time.time() - lap1
        avg_x, avg_y = await get_xy()
        if castCalibrated:
            sensorMagnitude = math.sqrt(avg_x ** 2 + avg_y ** 2) / ambientMag
            csv_writer.writerow(
                [time.strftime("%Y-%m-%d %H:%M:%S"), avg_x, avg_y, sourceAngle, sensorMagnitude, 0, 0, 0])
        else:
            sensorMagnitude = math.sqrt(avg_x ** 2 + avg_y ** 2)
            csv_writer.writerow(
                [time.strftime("%Y-%m-%d %H:%M:%S"), avg_x, avg_y, sourceAngle, sensorMagnitude, 0, 0, 0])
            ambientMag.append(sensorMagnitude)
        print("Moving right, Magnitude: " + str(sensorMagnitude))
        if castCalibrated:
            if sensorMagnitude > magThreshold:
                print('Flow detected, following')
                return True

        time.sleep(0.1)

    lap1 = time.time()
    while elapsed < goalCastTime / 2:
        tello.send_rc_control(-castSpeed, 0, 0, 0)  # Move left
        elapsed = time.time() - lap1
        avg_x, avg_y = await get_xy()
        if castCalibrated:
            sensorMagnitude = math.sqrt(avg_x ** 2 + avg_y ** 2) / ambientMag
            csv_writer.writerow(
                [time.strftime("%Y-%m-%d %H:%M:%S"), avg_x, avg_y, sourceAngle, sensorMagnitude, 0, 0, 0])
        else:
            sensorMagnitude = math.sqrt(avg_x ** 2 + avg_y ** 2)
            csv_writer.writerow(
                [time.strftime("%Y-%m-%d %H:%M:%S"), avg_x, avg_y, sourceAngle, sensorMagnitude, 0, 0, 0])
            ambientMag.append(sensorMagnitude)
        print("Moving left, Magnitude: " + str(sensorMagnitude))
        if castCalibrated:
            if sensorMagnitude > magThreshold:
                print('Flow detected, following')
                return True
        time.sleep(0.1)  # Small delay to prevent too frequent checks

    # Move Forward
    print('Moving forward...')
    elapsed = 0
    lap1 = time.time()
    while elapsed < forward_move_time:
        tello.send_rc_control(0, castSpeed, 0, 0)  # Move forward
        elapsed = time.time() - lap1
        avg_x, avg_y = await get_xy()
        if castCalibrated:
            sensorMagnitude = math.sqrt(avg_x ** 2 + avg_y ** 2) / ambientMag
            csv_writer.writerow(
                [time.strftime("%Y-%m-%d %H:%M:%S"), avg_x, avg_y, sourceAngle, sensorMagnitude, 0, 0, 0])
        else:
            sensorMagnitude = math.sqrt(avg_x ** 2 + avg_y ** 2)
            csv_writer.writerow(
                [time.strftime("%Y-%m-%d %H:%M:%S"), avg_x, avg_y, sourceAngle, sensorMagnitude, 0, 0, 0])
            ambientMag.append(sensorMagnitude)
            ambientMag = magTolerance * max(ambientMag)
            castCalibrated = True
            print(ambientMag)
            print("---------------------Cast Calibration Complete-------------------------------------")
        sensorMagnitude = math.sqrt(avg_x ** 2 + avg_y ** 2) / ambientMag
        print("Moving forward, Magnitude: " + str(sensorMagnitude))

        if castCalibrated:
            if sensorMagnitude > magThreshold:
                print('Flow detected, following')
                return True

        time.sleep(0.1)

    # Increment cast time for the next loop
    initial_cast_time += increment_time
    return False


# Define wait event
def wait():
    # tello.send_rc_control(0, 0, 0, 0)
    time.sleep(5)
    return True


async def read_data():
    data = await client.read_gatt_char(characteristic_uuid)
    if data:
        string_n = data.decode()
        ard_string = string_n.strip()
        ard_list = ard_string.split()
        x = float(ard_list[1])
        y = float(ard_list[2])
        return x, y


async def calibrate():
    global x_values, y_values, x_offset, y_offset, calibration_done, calibration_values_count, calMag, maxCalMag
    x, y = await read_data()
    while not calibration_done:
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
                print(x_offset, y_offset)
                print('Maximum Magnitude Detected: ' + str(maxCalMag))
    return x_offset, y_offset


async def get_xy():
    global x_offset, y_offset, x_buffer, y_buffer, num_points_for_averaging
    x, y = await read_data()
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
        avg_x = avg_x * 0.9869247406
        avg_y = avg_y * 0.9786229353
        return avg_x, avg_y
    return 0, 0


async def main():
    global tello, castState, turnState, runState, x_offset, y_offset, ambientMag, sensorMagnitude

    # Connect to the peripheral
    await client.connect()
    start_time = time.time()

    tello.connect()
    print(tello.query_battery())

    # Makes drone takeoff
    tello.takeoff()
    tello.send_rc_control(0, 0, 0, 0)
    time.sleep(2)
    tello.move_up(40)

    last_magnitude = None

    # initial calibration
    x_offset, y_offset = await calibrate()
    print('Initial Calibration Complete')
    while True:

        # if magnitude is greater than magnitude while drone is hovering without the fan on
        if await cast():
            castState = True
        if await turn():
            turnState = True
        # print(f"Average Sensor Magnitude: {sensor_magnitude}, Time: {elapsed_time}, Angle: {angle}")

        if keyboard.is_pressed('space'):
            print("SpaceBar pressed. Exiting.")
            tello.query_battery()
            tello.land()
            plot_source_angle_vs_time(filename)
            break
    await client.disconnect()


asyncio.run(main())

def plot_source_angle_vs_time(filename):
    times = []
    source_angles = []

    with open(filename, 'r') as csvfile:
        csv_reader = csv.DictReader(csvfile)
        for row in csv_reader:
            #times.append(float(row['Time']))
            source_angles.append(float(row['Sensor Magnitude']))

    plt.figure()
    plt.plot(source_angles, marker='o', linestyle='-')
    plt.title('Magnitude vs Unit')
    plt.xlabel('Units')
    plt.ylabel('Magnitude')
    plt.grid(True)
    plt.show()
