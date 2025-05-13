import math
import logging
import time
import os
import csv
from datetime import datetime
from math import atan2
from pickle import FALSE
from threading import Thread

from pynput import keyboard

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.motion_commander import MotionCommander
from cflib.crazyflie.log import LogConfig
from scipy.stats import false_discovery_control

# === STATE CONFIGURATION ===
manualCtrl = False
autoCtrl = False
flying = bool


# === DRONE CONFIGURATION ===
URI = 'radio://0/80/2M/E7E7E7E7E7'  # Ensure correct URI
directory_path = r"C:\Users\bridg\OneDrive\Documents\Research\Data"
base_filename = "imuWFlowLogger0.6ms"#"imu_loggerRun"
file_extension = ".csv"
DEFAULT_HEIGHT = 1

# === DRONE TEST PARAMETERS ===
prevTime = 0
Kp = 0.6
Ki = 0
Kd = 0.08
integral = 0
prevError = 0
prevTime = 0
currTime = 0
angleVar = 10
desiredAngle = 90
logger = 0

# === FILE HANDLING ===
file_number = 1
while True:
    csv_filename = f"{base_filename}{file_number}{file_extension}"
    full_path = os.path.join(directory_path, csv_filename)
    if not os.path.exists(full_path):
        break
    file_number += 1

# === LOGGING SETUP ===
class LoggerThread(Thread):
    def __init__(self, cf):
        super().__init__()
        self.cf = cf
        self.running = True
        self.data_call = {}
        self.calibration = {}

    def run(self):
        log_conf = LogConfig(name='Logger', period_in_ms=10)
        log_conf.add_variable('stateEstimate.vx', 'float')
        log_conf.add_variable('stateEstimate.vy', 'float')
        log_conf.add_variable('uart_logger.flowX', 'int16_t')
        log_conf.add_variable('uart_logger.flowY', 'int16_t')
        log_conf.add_variable('stabilizer.yaw', 'float')

        def log_data(timestamp, data, logconf):
            if self.running:
                now = datetime.now()
                if not self.calibration:
                    self.calibration = {
                    "Bx": data['uart_logger.flowX'],
                    "By": data['uart_logger.flowY']
                }

                self.data_call = {
                    "timestamp": now.microsecond,
                    "Vx": data['stateEstimate.vx'],
                    "Vy": data['stateEstimate.vy'],
                    "Bx": data['uart_logger.flowX'] - self.calibration["Bx"],
                    "By": data['uart_logger.flowY'] - self.calibration["By"],
                    "yaw": data['stabilizer.yaw']*180/math.pi
                }
                timestamp_list = now.month, now.day, now.hour, now.minute, now.second, now.microsecond
                writer.writerow({
                    "Month": now.month,
                    "Day": now.day,
                    "Hour": now.hour,
                    "Minute": now.minute,
                    "Second": now.second,
                    "Microsecond": now.microsecond,
                    "Vx": data['stateEstimate.vx'],
                    "Vy": data['stateEstimate.vy'],
                    "Bx": data['uart_logger.flowX'],
                    "By": data['uart_logger.flowY'],
                    "yaw": data['stabilizer.yaw']*180/math.pi
                })

        def log_error(logconf, msg):
            print(f"Logging error: {msg}")

        self.cf.log.add_config(log_conf)
        log_conf.data_received_cb.add_callback(log_data)
        log_conf.error_cb.add_callback(log_error)

        try:
            log_conf.start()
            while self.running:
                time.sleep(0.1)
            log_conf.stop()
        except Exception as e:
            print("Logging error:", e)

    def stop(self):
        self.running = False

# === KEYBOARD CONTROL ===
class KeyboardDrone:
    def __init__(self, mc):
        self.mc = mc
        self.velocity = 0.6
        self.ang_velocity = 120

    def on_press(self, key):
        try:
            if key.char == 'w':
                self.mc.start_forward(self.velocity)
                print("Drone should move forward")
            elif key.char == 's':
                self.mc.start_back(self.velocity)
                print("Drone should move backward")
            elif key.char == 'a':
                self.mc.start_left(self.velocity)
                print("Drone should move left")
            elif key.char == 'd':
                self.mc.start_right(self.velocity)
                print("Drone should move right")
            elif key.char == 'u':
                self.mc.take_off(0.5)
                print("Drone should take off")
            elif key.char == 'c':
                self.mc.start_down(self.velocity)
                print("Drone should move down")
            elif key.char == 'l':
                self.mc.land()
                print("Drone should land")
            elif key.char == 'q':
                self.mc.start_turn_left(self.ang_velocity)
                print("Drone should turn left")
            elif key.char == 'e':
                self.mc.start_turn_right(self.ang_velocity)
                print("Drone should turn right")

        except AttributeError:
            if key == keyboard.Key.space:
                self.mc.start_up(self.velocity)

    def on_release(self, key):
        self.mc.stop()
        print("stopping")
        if key == keyboard.Key.esc:
            return False  # Stop the listener

# === TEST CODE ===
def fly(scf, logger):
    global Kp, Ki, Kd, prevError, integral, start_time, current_time, prevTime, flying
    with (MotionCommander(scf, default_height=DEFAULT_HEIGHT) as mc):
        print(logger.data_call)
        current_time = logger.data_call["timestamp"]
        if prevTime is None or prevTime == 0:
            prevTime = current_time - 0.1
        sourceAngle = atan2(logger.data_call["Bx"], logger.data_call["By"])*180/math.pi
        sensorMag = math.sqrt(logger.data_call["Bx"]**2 + logger.data_call["By"]**2)
        heading = logger.data_call["yaw"]
        if heading < 0:
            heading = 360 + heading
        error = heading - sourceAngle
        if -270 <= error <= -180:
            error += 360
        delta_time = current_time - prevTime
        integral += error * delta_time
        derivative = (error - prevError) / delta_time  # if last_error is not None else 0
        command = int(Kp * error + Ki * integral + Kd * derivative)
        prevError = error
        prevTime = current_time

        if sensorMag > 2:  # 0.6*maxCal:
            # turnFlag
            sourceAngle = atan2(logger.data_call["Bx"], logger.data_call["By"]) * 180 / math.pi
            sensorMag = math.sqrt(logger.data_call["Bx"] ** 2 + logger.data_call["By"] ** 2)
            heading = logger.data_call["yaw"]
            if heading < 0: heading = 360 + heading
            error = heading - sourceAngle
            while abs(error) > angleVar:
                # if error is in Q1 or Q4 rotate right
                # updating logic to use source angle in if statement condition
                # Changing the line below to reflect positive values. If turning does not work, switch back

                if (270 <= sourceAngle <= 360) or (0 <= sourceAngle <= (desiredAngle - angleVar)):
                    mc.start_turn_right(error)
                    right_command = abs(command)
                    right_command = max(min(right_command, 100), 0)
                    # print(right_command)
                    print("Turning right, source angle & Magnitude: " + str(sourceAngle) + ':' + str(
                        sensorMag) + " error: " + str(error))
                    rightNow = datetime.now().strftime('%H:%M:%S')
                    time.sleep(0.05)

                # if error is in Q2 or Q3 rotate left
                # Changing the line below to negate above elif statement. Change back to else statement if logic fails
                elif (desiredAngle + angleVar) <= sourceAngle < 270:
                    mc.start_turn_left(error)
                    left_command = command
                    left_command = min(max(left_command, -100), 0)
                    print(left_command)
                    print("Turning left, source angle & Magnitude: " + str(sourceAngle) + ':' + str(
                        sensorMag) + " error: " + str(error))
                    rightNow = datetime.now().strftime('%H:%M:%S')
                    time.sleep(0.05)

            if abs(error) <= angleVar:  # if error is within desired threshold, hover
                print("Within Threshold. Moving towards detected flow at " + str(
                    sourceAngle) + ' degrees with a magnitude of ' + str(sensorMag))
                print("Stopping logger and landing...")
                logger.stop()
                time.sleep(1)  # Allow logger to finish
                mc.land()
                time.sleep(1)
                flying = False
        else:
            print("Searching for wind| Mag: " + str(sensorMag))
            rightNow = datetime.now().strftime('%H:%M:%S')
            


if __name__ == '__main__':
    cflib.crtp.init_drivers(enable_debug_driver=False)
    with open(full_path, mode="w", newline='') as csv_file:
        fieldnames = ["Month","Day","Hour","Minute","Second","Microsecond","Vx", "Vy", "Bx", "By", "yaw"]
        writer = csv.DictWriter(csv_file, fieldnames=fieldnames)
        writer.writeheader()
        with SyncCrazyflie(URI, cf=Crazyflie(rw_cache='./cache')) as scf:
            mc = MotionCommander(scf)
            logger = LoggerThread(scf.cf)
            logger.start()

            print("Type M for Manual Mode. Type A for Autonomous Mode. To enter Testing mode, press H")
            modeCtrl = input()
            if modeCtrl == "M":
                manualCtrl = True
                autoCtrl = False
            elif modeCtrl == "A":
                autoCtrl = True
                manualCtrl = False

            if manualCtrl:
                print("Ready for keyboard control. Press ESC to quit.")
                drone = KeyboardDrone(mc)
                with keyboard.Listener(on_press=drone.on_press, on_release=drone.on_release) as listener:
                    listener.join()
            elif autoCtrl:
                print("Ready for autonomous mode. Press ESC to quit.")
                drone = KeyboardDrone(mc)
            else:
                print("Entering test mode.")
                # Fly the drone
                flying = True
                while flying:
                    fly(scf, logger)
                
