from time import sleep
import time
import cflib.crtp
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.motion_commander import MotionCommander

URI = 'radio://0/80/2M'
DEFAULT_HEIGHT = 1


def fly(scf):
    with MotionCommander(scf, default_height=DEFAULT_HEIGHT) as mc:
        # Takeoff and hover for 60 seconds
        print("Taking off and hovering...")

        mc.forward(1)
        time.sleep(1)

        # Or turn
        mc.turn_left(90)
        time.sleep(1)
        mc.forward(1)
        # Land after 60 seconds
        print("Landing...")
        mc.land()


if __name__ == '__main__':
    # Initialize the low-level drivers
    cflib.crtp.init_drivers(enable_debug_driver=False)
    print("Connecting to Crazyflie...")

    # Connect to the Crazyflie
    with SyncCrazyflie(URI) as scf:
        print("Connection successful!")
        #print(f"Battery level: {scf.cf.param.get_value('pm.batteryLevel')}%")

        # Fly the drone
        fly(scf)
# -*- coding: utf-8 -*-
#
#     ||          ____  _ __
#  +------+      / __ )(_) /_______________ _____  ___
#  | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
#  +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
#   ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
#
#  Copyright (C) 2014 Bitcraze AB
#
#  Crazyflie Nano Quadcopter Client
#
#  This program is free software; you can redistribute it and/or
#  modify it under the terms of the GNU General Public License
#  as published by the Free Software Foundation; either version 2
#  of the License, or (at your option) any later version.
#
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU General Public License for more details.
# You should have received a copy of the GNU General Public License
# along with this program. If not, see <https://www.gnu.org/licenses/>.
"""
Simple example that scans for available Crazyflies with a certain address and lists them.
"""
