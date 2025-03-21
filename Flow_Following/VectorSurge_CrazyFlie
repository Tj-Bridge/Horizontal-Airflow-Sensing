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
import pandas
import matplotlib.pyplot as plt
import numpy as np
from numpy.f2py.crackfortran import endifs

# Import Crazyflie libraries
import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.motion_commander import MotionCommander
# from cflib.utils.multiranger import MultiRanger

# URI to the Crazyflie to connect to
URI = 'radio://0/80/2M/E7E7E7E7E7'  # Change this to match your Crazyflie URI

# Initialize drone/sensor parameters
angle = 0
sensorMagnitude = 0
castSpeed = 0.3  # m/s

# Initialize position-vector map (x_pos, y_pos, magnitude, angle)
# For now, We will plot magnitude vs. angle
flowMap = [(0.0, 0.0, 0.0, 0.0)]

# Initialize calibration and filtering
x_values = []
y_values = []
x_imu = 0
y_imu = 0
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

# Cast Variables
initial_cast_time = 4
increment_time = 2
forward_move_time = 0.5

# BLE setup
address = "18:2D:E3:60:9B:30"  # Change for your specific device
characteristic_uuid = "00002A56-0000-1000-8000-00805F9B34FB"

# Create a BleakClient object
client = BleakClient(address)


def get_unique_filename(base=r"C:\Users\bridg\OneDrive\Documents\Research", extension=".csv"):
    counter = 1
    while os.path.exists(f"{base}_{counter}{extension}"):
        counter += 1
    return f"{base}_{counter}{extension}"


filename = get_unique_filename()
csv_file = open(filename, mode='w', newline='')
csv_writer = csv.writer(csv_file)
csv_writer.writerow(['Time', 'Bx', 'By', 'Sensor Angle', 'Sensor Magnitude', 'Command', 'Error', 'State'])


# Define turn event
async def turn(mc):
    global angle, castState, sensorMagnitude, sourceAngle, magLimit, last_error, Ki, Kd, Kp, integral, prevTime, flowMap, ambientMag
    # print('in turn')
    if castState:
        while sensorMagnitude >= magThreshold:
            avg_x, avg_y = await get_xy()
            sourceAngle = (int(np.arctan2(avg_y, avg_x) * (180 / np.pi))) + 180 % 360
            sensorMagnitude = math.sqrt(avg_x ** 2 + avg_y ** 2) / ambientMag
            flowMap.append((avg_x, avg_y, sensorMagnitude, sourceAngle))
            error = desiredAngle - sourceAngle
            forward_command = 0.6  # m/s
            current_time = time.time()
            if prevTime is None or prevTime == 0:
                prevTime = current_time - 0.1
            error = desiredAngle - sourceAngle
            delta_time = current_time - prevTime
            integral += error * delta_time

            command = int(Kp * error + Ki * integral)

            last_error = error
            prevTime = current_time

            if abs(error) <= angle_threshold:  # if error is within desired threshold, hover
                print("Within Threshold. Moving towards detected flow at " + str(
                    sourceAngle) + ' degrees with a magnitude of ' + str(sensorMagnitude))
                mc.start_linear_motion(forward_command, 0, 0)  # Forward motion
                sensorMagnitude = math.sqrt(avg_x ** 2 + avg_y ** 2) / ambientMag
                flowMap.append((avg_x, avg_y, sensorMagnitude, sourceAngle))
                csv_writer.writerow(
                    [time.strftime("%Y-%m-%d %H:%M:%S"), avg_x, avg_y, sourceAngle, sensorMagnitude, 0, error, 2])
                time.sleep(0.1)

            elif (270 <= sourceAngle <= 360) or (0 <= sourceAngle <= (desiredAngle - angle_threshold)):
                right_command = abs(command) / 100  # Convert to angular rate in degrees/s, scale down
                right_command = max(min(right_command, 1.0), 0)  # Scale to Crazyflie range
                print(right_command)
                print("Turning right, source angle & Magnitude: " + str(sourceAngle) + ':' + str(
                    sensorMagnitude) + " error: " + str(error))
                mc.start_turn_right(right_command * 90)  # Convert to appropriate angular rate
                sensorMagnitude = math.sqrt(avg_x ** 2 + avg_y ** 2) / ambientMag
                flowMap.append((avg_x, avg_y, sensorMagnitude, sourceAngle))
                csv_writer.writerow(
                    [time.strftime("%Y-%m-%d %H:%M:%S"), avg_x, avg_y, sourceAngle, sensorMagnitude, right_command,
                     error, 1])
                time.sleep(0.05)

            elif (desiredAngle + angle_threshold) <= sourceAngle < 270:
                left_command = command / 100  # Convert to angular rate in degrees/s, scale down
                left_command = min(max(left_command, -1.0), 0)  # Scale to Crazyflie range
                left_command = abs(left_command)  # Make positive for turn_left function
                print(left_command)
                print("Turning left, source angle & Magnitude: " + str(sourceAngle) + ':' + str(
                    sensorMagnitude) + " error: " + str(error))
                mc.start_turn_left(left_command * 90)  # Convert to appropriate angular rate
                sensorMagnitude = math.sqrt(avg_x ** 2 + avg_y ** 2) / ambientMag
                flowMap.append((avg_x, avg_y, sensorMagnitude, sourceAngle))
                csv_writer.writerow(
                    [time.strftime("%Y-%m-%d %H:%M:%S"), avg_x, avg_y, sourceAngle, sensorMagnitude, left_command,
                     error, 1])
                time.sleep(0.05)
            last_error = error
            print(last_error)

            # End Condition for Flow Detection
            if sensorMagnitude > magLimit and abs(error) <= angle_threshold:
                flowMap.append((avg_x, avg_y, sensorMagnitude, sourceAngle))
                csv_writer.writerow(
                    [time.strftime("%Y-%m-%d %H:%M:%S"), avg_x, avg_y, sourceAngle, sensorMagnitude, 0, error, 4])
                print("Maximum magnitude exceeded ... ending run")
                mc.land()
                await client.disconnect()
                break
    else:
        return False
    return True


# Define the cast event
async def cast(mc):
    global sensorMagnitude, magThreshold, castCalibrated, ambientMag, magTolerance, initial_cast_time, increment_time, forward_move_time
    # Initialize Variable as list before calibrating
    if not castCalibrated:
        ambientMag = []

    lap1 = time.time()
    avg_x, avg_y = await get_xy()
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
    while elapsed < goalCastTime / 2:
        mc.start_linear_motion(0, -castSpeed, 0)  # Move left (negative y-velocity)
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
        mc.start_linear_motion(castSpeed, 0, 0)  # Move forward
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
        mc.start_linear_motion(0, castSpeed, 0)  # Move right (positive y-velocity)
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

    # Move Left (again)
    lap1 = time.time()
    elapsed = 0  # Reset elapsed time for this segment
    while elapsed < goalCastTime / 2:
        mc.start_linear_motion(0, -castSpeed, 0)  # Move left (negative y-velocity)
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

    # Move Forward (again)
    print('Moving forward...')
    elapsed = 0
    lap1 = time.time()
    while elapsed < forward_move_time:
        mc.start_linear_motion(castSpeed, 0, 0)  # Move forward
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


async def read_data():
    data = await client.read_gatt_char(characteristic_uuid)
    if data:
        string_n = data.decode()
        ard_string = string_n.strip()
        ard_list = ard_string.split()
        x = float(ard_list[1])
        y = float(ard_list[2])
        x_IMU = float(ard_list[3])
        y_IMU = float(ard_list[4])
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
    global castState, turnState, runState, x_offset, y_offset, ambientMag, sensorMagnitude

    # Initialize the low-level drivers
    cflib.crtp.init_drivers()

    # Connect to the peripheral
    await client.connect()
    start_time = time.time()

    # Initialize Crazyflie
    with SyncCrazyflie(URI) as scf:
        with MotionCommander(scf) as mc:
            print(f"Connected to Crazyflie at {URI}")
            print(f"Battery level: {scf.cf.param.get_value('pm.vbat')}V")

            # Takeoff and hover
            # Note: MotionCommander automatically performs takeoff
            mc.up(0.4)  # Move up 40cm
            time.sleep(2)

            last_magnitude = None

            # initial calibration
            x_offset, y_offset = await calibrate()
            print('Initial Calibration Complete')
            while True:
                # if magnitude is greater than magnitude while drone is hovering without the fan on
                if await cast(mc):
                    castState = True
                if await turn(mc):
                    turnState = True

                if keyboard.is_pressed('space'):
                    print("SpaceBar pressed. Exiting.")
                    print(f"Battery level: {scf.cf.param.get_value('pm.vbat')}V")
                    mc.land()
                    plot_source_angle_vs_time(filename)
                    break
    await client.disconnect()


def plot_magnitude_vs_time(array):
    magnitudes = []
    angles = []
    for row in array:
        magnitudes.append(row[2])
    for row in array:
        angles.append(row[3])
    plt.figure()
    plt.plot(magnitudes)
    plt.title('Magnitude vs Unit')
    plt.xlabel('Units')
    plt.ylabel('Magnitude')


def plot_source_angle_vs_time(filename):
    times = []
    source_angles = []

    with open(filename, 'r') as csvfile:
        csv_reader = csv.DictReader(csvfile)
        for row in csv_reader:
            # times.append(float(row['Time']))
            source_angles.append(float(row['Sensor Magnitude']))

    plt.figure()
    plt.plot(source_angles, marker='o', linestyle='-')
    plt.title('Magnitude vs Unit')
    plt.xlabel('Units')
    plt.ylabel('Magnitude')
    plt.grid(True)
    plt.show()


if __name__ == '__main__':
    asyncio.run(main())
