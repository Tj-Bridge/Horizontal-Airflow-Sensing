import logging
import time
import os
import csv
from datetime import datetime
from threading import Thread

from pynput import keyboard

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.motion_commander import MotionCommander
from cflib.crazyflie.log import LogConfig

# === CONFIGURATION ===
URI = 'radio://0/80/2M/E7E7E7E7E7'  # Ensure correct URI
directory_path = r"C:\Users\ltjth\Documents\Research\VelocityLogs"
base_filename = "imuWFlowLogger0.6ms"#"imu_loggerRun"
file_extension = ".csv"

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

    def run(self):
        log_conf = LogConfig(name='Logger', period_in_ms=10)
        log_conf.add_variable('stateEstimate.vx', 'float')
        log_conf.add_variable('stateEstimate.vy', 'float')
        log_conf.add_variable('uart_logger.flowX', 'int16_t')
        log_conf.add_variable('uart_logger.flowY', 'int16_t')





        def log_data(timestamp, data, logconf):
            if self.running:
                now = datetime.now()
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
                    "By": data['uart_logger.flowY']

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





if __name__ == '__main__':
    cflib.crtp.init_drivers(enable_debug_driver=False)
    with open(full_path, mode="w", newline='') as csv_file:
        fieldnames = ["Month","Day","Hour","Minute","Second","Microsecond","Vx", "Vy", "Bx", "By"]
        writer = csv.DictWriter(csv_file, fieldnames=fieldnames)
        writer.writeheader()
        with SyncCrazyflie(URI, cf=Crazyflie(rw_cache='./cache')) as scf:
            mc = MotionCommander(scf)
            logger = LoggerThread(scf.cf)
            logger.start()


            print("Ready for keyboard control. Press ESC to quit.")
            drone = KeyboardDrone(mc)
            with keyboard.Listener(on_press=drone.on_press, on_release=drone.on_release) as listener:
                listener.join()

            print("Stopping logger and landing...")
            logger.stop()
            time.sleep(1)  # Allow logger to finish
            mc.land()
            time.sleep(1)
