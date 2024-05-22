import rospy
import time
import datetime
import numpy as np
import rospy_crazyflie.crazyflie_client as crazyflie_client
from rospy_crazyflie.crazyflie_client import CrazyflieClient
from rospy_crazyflie.srv import *
from rospy_crazyflie.msg import *
from std_msgs.msg import String

def ctrl_callback(data):
    # this callback takes in keyboard data and updates flight commands to crazyflie
    global vel, end_flag, eag_flag, auto_flag,client # list global variables I will be using here

    if data.data == "+": # go up
        client.up(0.05)
        print('up')
    if data.data == "-": # go down
        client.down(0.05)
        print('down')
    if data.data == "8": # go forward
        vel[0] += 0.1 # update x velocity
    if data.data == "4": # go left
        vel[1] += 0.1 # update y velocity
    if data.data == "2": # go back
        vel[0] -= 0.1 # update x velocity
    if data.data == "6": # go right
        vel[1] -= 0.1 # update y velocity
    if data.data == "7": # rotate left
        if vel[3] == 0:
            vel[2] += 10
        else:
            vel[3] -= 10
    if data.data == "9": # rotate right
        if vel[2] == 0:
            vel[3] += 10
        else:
            vel[2] -= 10
    if data.data == "5": # stop all movement
        vel[0] = 0.0
        vel[1] = 0.0
        vel[2] = 0.0
        vel[3] = 0.0
    if data.data == "0": # stop all movement and end program
        vel[0] = 0.0
        vel[1] = 0.0
        vel[2] = 0.0
        vel[3] = 0.0
        end_flag = True
    if data.data == "1": # keyboard trigger smell
        eag_flag = True
        print("keyboard trigger smell")
    if data.data == "3": # toggle cast-and-surge OR manual flight
        if auto_flag == True:
            auto_flag = False
            print("auto flight {}".format(auto_flag))
        elif auto_flag == False:
            auto_flag = True
            eag_flag = False
            print("auto flight {}".format(auto_flag))


def eag_callback(data, timestamp):
    # This callback logs data from EAG and will set flag if it smells (above threshold)
    global eag, eag_flag, threshold, eag_list, pub # list global variables

    eag = float(data['eagdeck.eagdeck'])
    pub.publish(str(data['eagdeck.eagdeck']))
    eag_list.append(eag)
    # print("eag: {}".format(eag))
    if (eag > (1.5+threshold)*4096/3):# or (eag < (1.5-threshold)*4096/3):
        # print("eag: {}".format(eag))
        eag_flag = True

def pos_callback(data, timestamp):
    # this callback reads position estimate of crazyflie
    global pos # list global variable
    pos[0] = float(data['stateEstimate.x'])
    pos[1] = float(data['stateEstimate.y'])
    pos[2] = float(data['stateEstimate.z'])
    pos[3] = float(data['stabilizer.yaw'])
    pos[4] = float(data['stabilizer.pitch'])
    pos[5] = float(data['stabilizer.roll'])

def sen_callback(data, timestamp):
    # this callback reads data from sensors
    global dist_flag, sen # list global vaiables
    sen[0] = float(data['range.left'])/1000. # change data from cm to m
    sen[1] = float(data['range.right'])/1000.
    sen[2] = float(data['range.front'])/1000.
    sen[3] = float(data['range.back'])/1000.
    sen[4] = float(data['range.up'])/1000.
    sen[5] = float(data['range.zrange'])/1000.

    # Uncomment this section if you want to land by hovering hand above crazyflie
    # if float(data['range.up']) < 150:
    #     dist_flag = True
    #     print('detect motion above, landing now')


if __name__ == "__main__":
    rospy.init_node('eag')
    rospy.Subscriber('controls', String, ctrl_callback)
    pub = rospy.Publisher('eag_pub', String, queue_size=10)
    rate = rospy.Rate(10) # 10hz

    # Set up initial values of variables
    fan_demo = True
    MIN = 0.2 # minimum distance to walls to trigger stop motion in that direction
    if fan_demo == True:
        state = -1
    else:
        state = 3 # inital state (casting half left)
    pos = [0,0,0,0,0,0] # x,y,z,yaw,pitch,roll
    sen = [0,0,0,0,0,0] # l,r,f,b,u,d
    vel = [0,0,0,0] # x_vel,y_vel,left_rate,right_rate
    eag = 0 # inital eag value
    eag_list = []
    dist = 0 # initial distance to travel
    eag_flag = False # True when smell
    threshold = 0.2 # Threshold for eag signal in volts above/below baseline 1.5V
    auto_flag = False # True when switching from manual flight to cast-and-surge
    dist_flag = False # True when object is close to top of drone
    end_flag = False # True when program is triggered to end
    inc = 0 # increment to increase casting by

    # Set up data file
    timenow = str(datetime.datetime.now().strftime('%Y_%m_%d_%H_%M_%S'))
    file_name = "data/" + timenow + "_data.txt"
    f = open(file_name,"a")
    f.write("time: {}\n".format(datetime.datetime.now()))
    print("time: {}".format(datetime.datetime.now()))
    f.write("sensors: l,r,f,b,u,d\n")
    f.write("pose: x,y,z,yaw,pitch,roll\n")
    f.write("eag, threshold {}\n".format(threshold))

    # Connect to a Crazyflie on the server
    crazyflies = crazyflie_client.get_crazyflies('/crazyflie_server')
    client = CrazyflieClient(crazyflies[0])

    # Add a log for eag
    name = 'eag_log'
    variables = [crazyflie_client.LogVariable('eagdeck.eagdeck', 'float')]
    period_ms = 10
    client.add_log_config(
        name,
        variables,
        period_ms,
        callback=eag_callback
    )
    # Add a log for position
    name2 = 'pos_log'
    variables2 = [crazyflie_client.LogVariable('stateEstimate.x', 'float'),
                crazyflie_client.LogVariable('stateEstimate.y', 'float'),
                crazyflie_client.LogVariable('stateEstimate.z', 'float'),
                crazyflie_client.LogVariable('stabilizer.yaw', 'float'),
                crazyflie_client.LogVariable('stabilizer.pitch', 'float'),
                crazyflie_client.LogVariable('stabilizer.roll', 'float')]
    period_ms2 = 10
    client.add_log_config(
        name2,
        variables2,
        period_ms2,
        callback=pos_callback
    )
    # Add a log for sensor data
    name3 = 'sen_log'
    variables3 = [crazyflie_client.LogVariable('range.left', 'float'),
                crazyflie_client.LogVariable('range.right', 'float'),
                crazyflie_client.LogVariable('range.front', 'float'),
                crazyflie_client.LogVariable('range.back', 'float'),
                crazyflie_client.LogVariable('range.up', 'float'),
                crazyflie_client.LogVariable('range.zrange', 'float')]
    period_ms3 = 10
    client.add_log_config(
        name3,
        variables3,
        period_ms3,
        callback=sen_callback
    )

    # Start flying
    time.sleep(2) # wait to stabilize
    client.take_off(height=.3)
    client.wait() # wait to take off
    time.sleep(5) # wait to stabilize

    # client.set_param('pid_rate.yaw_ki', '1.00')
    # client.set_param('pid_attitude.yaw_kp', '0.00')
    # client.set_param('pid_attitude.yaw_ki', '0.00')
    time.sleep(5) # wait to orient upwind

    # Main loop, will continue until signaled to land
    while not end_flag and not dist_flag:
        if auto_flag == True: # execute cast-and-surge
            f.write("starting cast and surge\n")
            f.write("starting cast and surge\n")
            f.write("starting cast and surge\n")
            # check if obstacles
            if sen[2] < MIN:
                print("*****obstacle ahead at {} m, stopping".format(sen[2]))
                client.stop()
                # end_flag = True
                state = 1
            elif fan_demo == True: # dont care about side obstacles during fan demo
                pass
            elif sen[0] < MIN and (state == 1.5 or state == 3.5):
                print("*****obstacle to left at {} m".format(sen[0]))
                client.stop()
                state = 2 # switch to going right
            elif sen[1] < MIN and state == 2.5:
                print("*****obstacle to right at {} m".format(sen[1]))
                client.stop()
                state = 1 # switch to going left

            # check if surge
            if (eag_flag == True) and (end_flag != True):
                print("*****encountered smell")
                if state != 0: # if doing something else (other than surging), stop
                    client.stop()
                state = 0 # switch to surging state
                eag_flag = False

            #if action has finished, start new action
            casting_vel = 0.15
            if state*2 % 2 == 0: # not an in progress state (in progress ends in .5)
                if state == 0: # start surging
                    client.start_linear_motion(casting_vel, 0.0, 0.0)
                    if fan_demo == True:
                        state = -1.5
                    else:
                        state = 0.5 # set state to in progress surging
                    dist = 0.25 # amount to move forward
                    print("*****surging now {}m".format(dist))
                elif state == -1: # hovering
                    pass
                elif state == 1: # start going left
                    client.start_linear_motion(0.0, casting_vel, 0.0)
                    state = 1.5 # set state to in progress going left
                    inc += 0.0625
                    dist = 0.125 + inc # amount to move left
                    print("*****going left now {}m".format(dist))
                elif state == 2: # start going right
                    client.start_linear_motion(0.0, -casting_vel, 0.0)
                    state = 2.5 # set state to in progress going right
                    inc += 0.0625
                    dist = 0.125 + inc # amount to move right
                    print("*****going right now {}m".format(dist))
                else: # start going half left
                    client.start_linear_motion(0.0, casting_vel, 0.0)
                    state = 3.5 # set state in progress going half left
                    inc = 0.0625
                    dist = 0.125 # amount to move left
                    print("*****switching to casting, going left now {}m".format(dist))
                start_pos = [pos[0],pos[1]] # keep track of starting position

            # if have moved correct distance, stop and switch state
            if np.sqrt((start_pos[0] - pos[0])**2 + (start_pos[1] - pos[1])**2) > dist:
                client.stop()
                if state == -1.5: #ending surge (only happens if in fan_demo mode)
                    print("*****state from surge to hover")
                    state = -1 # begin hover
                elif state == 1.5: #end left
                    print("*****state from left to right")
                    state = 2 #begin right
                elif state == 2.5: #end right
                    print("*****state from right to left")
                    state = 1 #begin left
                elif state == 3.5: #end half left
                    print("*****state from half left to right")
                    state = 2 #begin right
                else: #end surging
                    print("*****state from surge to half left")
                    state = 3 #begin half left motion

        else: # manual flight controlled via keypress, not doing cast-and-surge
            if vel[2] != 0:
                client.start_turn_left(vel[2])
            elif vel[3] != 0:
                client.start_turn_right(vel[3])
            else:
                client.start_linear_motion(vel[0], vel[1], 0.0)
            pass
        f.write("time: {}".format(datetime.datetime.now()))
        f.write("sensors: {}\n".format(sen))
        f.write("pose: {}\n".format(pos))
        f.write("eag: {}\n".format(eag_list))
        if len(eag_list) > 0:
            print(max(eag_list))
        eag_list = []
        time.sleep(0.1) # wait 1/10 of a second, then repeat while loop
    # end while loop

    # print reason for landing
    print("end triggered: {}".format(end_flag))
    print("top sensor triggered: {}".format(dist_flag))
    client.stop()
    client.land()
    client.wait()

    f.close()
    del client
    rospy.signal_shutdown(0)

