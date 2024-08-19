import asyncio
from bleak import BleakClient
import time
import csv
import keyboard
import os
import math
from djitellopy import Tello
import numpy as np

inFlow = False
turnState = False
hoverState = False
#desiredAngle = 90
angle_threshold = 5
# PID settings and state
Kp = 0.95
Ki = 0.01
Kd = 0.05
desiredAngleArr = [90, 180, 270, 360]
angle_index = 0

integral = 0
last_error = None
last_time = None

# Determine a unique filename for the CSV file
def get_unique_filename(base="AugRotation_Back", extension=".csv"):
    counter = 1
    while os.path.exists(f"{base}_{counter}{extension}"):
        counter += 1
    return f"{base}_{counter}{extension}"

filename = get_unique_filename()
csv_file = open(filename, mode='w', newline='')
csv_writer = csv.writer(csv_file)
csv_writer.writerow(['Time', 'Yaw', 'Kp', 'Ki', 'Kd', 'Command', 'Error', 'Desired Angle', 'Sensor Magnitude'])

def turn(desiredAngle, sourceAngle, sensor_magnitude):
    global tello, angle_threshold, Kp, Ki, Kd, last_error, last_time, integral
    #
    # current_time = time.time()
    # if last_time is None:
    #     last_time = current_time

    error = desiredAngle - sourceAngle
    # delta_time = current_time - last_time
    #
    # integral += error * delta_time
    # derivative = (error - last_error) / delta_time if last_error is not None else 0
    #
    # command = int(Kp * error + Ki * integral + Kd * derivative)
    #
    # last_error = error
    # last_time = current_time

    if abs(error) <= angle_threshold:  # if error is within desired threshold, hover
        print("Within Threshold   error: " + str(abs(error)) + " sourceAngle: " + str(sourceAngle))
        hover()
        command = 0
        csv_writer.writerow(
            [time.strftime("%Y-%m-%d %H:%M:%S"), sourceAngle, Kp, Ki, Kd, command, error, desiredAngle,
             sensor_magnitude])

        # if error is in Q1 or Q4 rotate right
        # updating logic to use source angle in if statemetnt condition
    elif 0 <= sourceAngle <= (desiredAngle - angle_threshold) or -270 <= sourceAngle <= -180:
        right_command = int(Kp * abs(error))
        right_command = max(min(right_command, 100), -100)
        print("R: " + str(right_command) + "| Mag: " + str(sensor_magnitude) + " Error: " + str(abs(error)))
        tello.send_rc_control(0, 0, 0, right_command)  # velocity-based
        csv_writer.writerow(
            [time.strftime("%Y-%m-%d %H:%M:%S"), sourceAngle, Kp, Ki, Kd, right_command, error, desiredAngle,
             sensor_magnitude])
        time.sleep(0.1)

    # if error is in Q2 or Q3 rotate left
    else:
        left_command = -int(Kp * abs(error))
        left_command = max(min(left_command, 100), -100)
        print("L: " + str(left_command) + "| Mag: " + str(sensor_magnitude) + " Error: " + str(abs(error)))
        tello.send_rc_control(0, 0, 0, left_command)  # velocity-based
        csv_writer.writerow(
            [time.strftime("%Y-%m-%d %H:%M:%S"), sourceAngle, Kp, Ki, Kd, left_command, error, desiredAngle,
             sensor_magnitude])
        time.sleep(0.1)

    # Log data

def hover():
    #global tello
    print("Hovering")
    tello.send_rc_control(0, 0, 0, 0)
    time.sleep(0.1)
    #hoverState = True
    #return hoverState

async def main():
    global tello
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

    #Makes drone takeoff
    tello.takeoff()
    time.sleep(2)

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

            if not calibration_done:
                if len(x_values) < calibration_values_count:
                    x_values.append(x)
                    y_values.append(y)
                    if len(x_values) == calibration_values_count:

                        x_offset = sum(x_values) / calibration_values_count
                        y_offset = sum(y_values) / calibration_values_count
                        calMag = [math.sqrt((xi-x_offset)**2 + (yi-y_offset)**2) for xi, yi in zip(x_values, y_values)]
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

                #calibration
                avg_x = avg_x*0.7536477847
                avg_y = avg_y*1.272200653

                #core variables
                sourceAngle = (int(np.arctan2(avg_y, avg_x) * (180 / np.pi))) % 360
                sensor_magnitude = math.sqrt(avg_x**2 + avg_y**2)

                if keyboard.is_pressed('a'):
                    if desiredAngle == 90:
                        desiredAngle = 270#input("Input desired angle ")
                        print("270 Desired")
                    elif desiredAngle == 270:
                        desiredAngle = 90
                        print("90 Desired")
                # if magnitude is greater than magnitude while drone is hovering without the fan on
                if sensor_magnitude >= maxCalMag:
                    #vinFlow = Trueaaaaa
                    print("Wind detected")
                    turn(desiredAngle, sourceAngle, sensor_magnitude)

                else:
                    print("Searching for wind| Mag: " + str(sensor_magnitude))
                    hover()
                #print(f"Average Sensor Magnitude: {sensor_magnitude}, Time: {elapsed_time}, Angle: {angle}")

        if keyboard.is_pressed('space'):
            print("Spacebar pressed. Exiting.")
            tello.query_battery()
            tello.land()
            break
        # if keyboard.is_pressed('a'):
        #     left += 1
        #     #tello.send_rc_control(0, 0, 0, left)
        # if keyboard.is_pressed('d'):
        #     right -= 1
        #     #tello.send_rc_control(0, 0, 0, right)


    await client.disconnect()

asyncio.run(main())
