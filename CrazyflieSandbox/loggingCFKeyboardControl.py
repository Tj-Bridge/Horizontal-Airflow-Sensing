import logging
import threading
import time
from pynput import keyboard

import cflib.crtp
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.motion_commander import MotionCommander

URI = 'radio://0/80/2M'
logging.basicConfig(level=logging.ERROR)


class KeyboardDrone:
    def __init__(self, mc):
        self.mc = mc
        self.max_velocity = 0.4
        self.accel_rate = 0.05
        self.update_interval = 0.1

        self.active_keys = set()
        self.target_x = 0
        self.target_y = 0
        self.target_z = 0
        self.current_x = 0
        self.current_y = 0
        self.current_z = 0

        self.turning = 0  # -1 for left, 1 for right, 0 for no turning
        self.turn_speed = 180  # degrees per second

        self.running = False
        self.motion_thread = None
        print("Press 'u' to take off, 'l' to land.")

    def _motion_loop(self):
        while self.running:
            self.current_x = self._approach(self.current_x, self.target_x)
            self.current_y = self._approach(self.current_y, self.target_y)
            self.current_z = self._approach(self.current_z, self.target_z)
            self.mc.start_linear_motion(self.current_x, self.current_y, self.current_z)

            if self.turning == -1:
                self.mc.start_turn_left(self.turn_speed)
            elif self.turning == 1:
                self.mc.start_turn_right(self.turn_speed)
            else:
                self.mc._stop_turn()  # private call, but necessary here

            time.sleep(self.update_interval)

    def _approach(self, current, target):
        if abs(target - current) < self.accel_rate:
            return target
        return current + self.accel_rate * (1 if target > current else -1)

    def update_targets(self):
        self.target_x = 0
        self.target_y = 0
        self.target_z = 0

        if 'w' in self.active_keys:
            self.target_x += self.max_velocity
        if 's' in self.active_keys:
            self.target_x -= self.max_velocity
        if 'd' in self.active_keys:
            self.target_y += self.max_velocity
        if 'a' in self.active_keys:
            self.target_y -= self.max_velocity
        if 'p' in self.active_keys:
            self.target_z += self.max_velocity
        if 'c' in self.active_keys:
            self.target_z -= self.max_velocity

    def on_press(self, key):
        try:
            k = key.char.lower()
        except AttributeError:
            return

        if k == 'u':
            self.mc.take_off(0.5)
            time.sleep(1.5)
            if not self.running:
                self.running = True
                self.motion_thread = threading.Thread(target=self._motion_loop)
                self.motion_thread.start()
        elif k == 'l':
            self.stop()
            self.mc.land()
        elif k in {'w', 'a', 's', 'd', 'p', 'c'}:
            if k not in self.active_keys:
                self.active_keys.add(k)
                self.update_targets()
        elif k == 'q':
            self.turning = -1
        elif k == 'e':
            self.turning = 1

    def on_release(self, key):
        try:
            k = key.char.lower()
            if k in self.active_keys:
                self.active_keys.remove(k)
                self.update_targets()
            elif k in {'q', 'e'}:
                self.turning = 0
        except AttributeError:
            pass

    def stop(self):
        if self.running:
            self.running = False
            if self.motion_thread is not None:
                self.motion_thread.join()
            self.mc.stop()
            self.mc._stop_turn()


if __name__ == '__main__':
    cflib.crtp.init_drivers(enable_debug_driver=False)

    with SyncCrazyflie(URI) as scf:
        mc = MotionCommander(scf)
        drone = KeyboardDrone(mc)

        try:
            with keyboard.Listener(
                on_press=drone.on_press,
                on_release=drone.on_release
            ) as listener:
                listener.join()
        finally:
            drone.stop()
