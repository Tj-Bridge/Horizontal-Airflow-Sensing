import asyncio
from bleak import BleakClient
import time
import csv
import keyboard
import os
import math
from djitellopy import Tello
import numpy as np
from datetime import datetime
import matplotlib.pyplot as plt  # Import matplotlib for plotting
inFlow = False
turnState = False
hoverState = False
#desiredAngle = 90
angle_threshold = 1
# PID settings and state
#.8 0 .1 i think
Kp = 0.6
Ki = 0
Kd = 0.08
desiredAngleArr = [90, 180, 270, 360]
angle_index = 0


integral = 0
last_error = 0
last_time = 0
prevError = 0
prevTime = 0
currTime = 0
forward_command = 0

start_time_flag = 0

# Determine a unique filename for the CSV file
def get_unique_filename(base=r"C:\Users\ltjth\Documents\Research\velocitytrials", extension=".csv"):
    counter = 1
    while os.path.exists(f"{base}_{counter}{extension}"):
        counter += 1
    return f"{base}_{counter}{extension}"

filename = get_unique_filename()
csv_file = open(filename, mode='w', newline='')
csv_writer = csv.writer(csv_file)
csv_writer.writerow(['Time','X','Y' 'Source Angle', 'Sensor Magnitude', 'Drone Yaw','Drone X Speed'])



def hover():
    #global tello
    print("Hovering")
    tello.send_rc_control(0, 0, 0, 0)
    time.sleep(0.05)
    #hoverState = True
    #return hoverState

async def main():
    global tello,start_time,start_time_flag
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
    hover()
    tello.move_up(20)
    time.sleep(3)

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
    num_points_for_averaging = 20
    calibration_values_count = 20

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
                        print(0.6*maxCalMag)
                        # print("Average: " + str(sum(calMag) / calibration_values_count))
                        calibration_done = True

                        start_time = time.time()

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
                avg_x = avg_x * 0.9795326160
                avg_y = avg_y * 0.9935092312

                #core variables
                sourceAngle = (np.arctan2(avg_y, avg_x) * (180 / np.pi)) % 360
                sensor_magnitude = math.sqrt(avg_x**2 + avg_y**2)

                elapsed_time = time.time() - start_time

                tello.send_rc_control(0,20,0,0)
                XSpeed = tello.get_speed_x()
                print("X speed: " + str(XSpeed))
                csv_writer.writerow(
                    [elapsed_time, avg_x, avg_y, sourceAngle,sensor_magnitude, tello.get_yaw(),XSpeed])
                if elapsed_time > 20:
                    tello.query_battery()
                    tello.send_rc_control(0,0,0,0)
                    time.sleep(2)
                    tello.land()
                    tello.end()
                    csv_file.close()
                    plot_source_angle_vs_time(filename)
                    break



        if keyboard.is_pressed('space'):
            print("Spacebar pressed. Exiting.")
            tello.query_battery()
            tello.land()
            tello.end()
            csv_file.close()
            plot_source_angle_vs_time(filename)
            break
        # if keyboard.is_pressed('a'):
        #     left += 1
        #     #tello.send_rc_control(0, 0, 0, left)
        # if keyboard.is_pressed('d'):
        #     right -= 1
        #     #tello.send_rc_control(0, 0, 0, right)


    await client.disconnect()
    tello.end()

    # Open the CSV file and plot the data


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
    plt.title('Error vs Unit')
    plt.xlabel('Units')
    plt.ylabel('Error(degrees)')
    plt.grid(True)
    plt.show()

asyncio.run(main())
