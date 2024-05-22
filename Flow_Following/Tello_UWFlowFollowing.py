from djitellopy import Tello
import time


# def eag_callback(data, timestamp):
    # This callback logs data from EAG and will set flag if it smells (above threshold)
    #   global eag, eag_flag, threshold, eag_list, pub  # list global variables

    #   eag = float(data['eagdeck.eagdeck'])
    #   pub.publish(str(data['eagdeck.eagdeck']))
    #   eag_list.append(eag)
    # print("eag: {}".format(eag))
    #   if eag > (1.5+threshold)*4096/3:  # or (eag < (1.5-threshold)*4096/3):
        # print("eag: {}".format(eag))
        #   eag_flag = True


# def pos_callback(data, timestamp):
    # this callback reads position estimate of crazyflie
    #   global pos  # list global variable
    #   pos[0] = float(data['stateEstimate.x'])
    #   pos[1] = float(data['stateEstimate.y'])
    #   pos[2] = float(data['stateEstimate.z'])
    #   pos[3] = float(data['stabilizer.yaw'])
    #   pos[4] = float(data['stabilizer.pitch'])
    #   pos[5] = float(data['stabilizer.roll'])


# def sen_callback(data, timestamp):
    # this callback reads data from sensors
    #   global dist_flag, sen  # list global variables
    #   sen[0] = float(data['range.left'])/1000.  # change data from cm to m
    #   sen[1] = float(data['range.right'])/1000.
    #   sen[2] = float(data['range.front'])/1000.
    #   sen[3] = float(data['range.back'])/1000.
    #   sen[4] = float(data['range.up'])/1000.
    #   sen[5] = float(data['range.zrange'])/1000.

    # Uncomment this section if you want to land by hovering hand above crazyflie
    # if float(data['range.up']) < 150:
    #     dist_flag = True
    #     print('detect motion above, landing now')


if __name__ == "__main__":
    # ROS Setup
    #   rospy.init_node('eag')
    #   rospy.Subscriber('controls', String, ctrl_callback)
    #   pub = rospy.Publisher('eag_pub', String, queue_size=10)
    #   rate = rospy.Rate(10) # 10hz

    # Set up initial values of variables
    fan_demo = True
    moveDist = 2
    MIN = 0.2  # minimum distance to walls to trigger a stop in motion in that direction
    if fan_demo:
        state = -1
    else:
        state = 3  # initial state (casting half left)
    pos = [0, 0, 0, 0, 0, 0]  # x,y,z,yaw,pitch,roll
    sen = [0, 0, 0, 0, 0, 0]  # l,r,f,b,u,d
    vel = [0, 0, 0, 0]  # x_vel,y_vel,left_rate,right_rate
    eag = 0  # initial eag value
    eag_list = []
    dist = 0  # initial distance to travel
    eag_flag = False  # True when smell
    threshold = 0.2  # Threshold for eag signal in volts above/below baseline 1.5V
    auto_flag = True  # True when switching from manual flight to cast-and-surge
    dist_flag = False  # True when object is close to top of drone
    end_flag = False  # True when program is triggered to end
    inc = 0  # increment to increase casting by

    # Set up data file
    #   timeNow = str(datetime.datetime.now().strftime('%Y_%m_%d_%H_%M_%S'))
    #   file_name = "data/" + timeNow + "_data.txt"
    #   f = open(file_name, "a")
    #   f.write("time: {}\n".format(datetime.datetime.now()))
    #   print("time: {}".format(datetime.datetime.now()))
    #   f.write("sensors: left,right,forward,backward,up,down\n")
    #   f.write("pose: x,y,z,yaw,pitch,roll\n")
    #   f.write("eag, threshold {}\n".format(threshold))

    # Connect to a Crazyflie on the server
    #   crazyflies = crazyflie_client.get_crazyflies('/crazyflie_server')
    #   client = CrazyflieClient(crazyflies[0])
    tello = Tello()
    tello.connect()

    # Add a log for eag
    name = 'eag_log'
    variables = [tello.LOGGER.log('eagdeck.eagdeck', 'float')]
    period_ms = 10
    tello.LOGGER.add_log_config(
        name,
        variables,
        period_ms,
        # callback=eag_callback
    )
    # Add a log for position
    name2 = 'pos_log'
    variables2 = [tello.LOGGER.LogVariable('stateEstimate.x', 'float'),
                  tello.LOGGER.LogVariable('stateEstimate.y', 'float'),
                  tello.LOGGER.LogVariable('stateEstimate.z', 'float'),
                  tello.LOGGER.LogVariable('stabilizer.yaw', 'float'),
                  tello.LOGGER.LogVariable('stabilizer.pitch', 'float'),
                  tello.LOGGER.LogVariable('stabilizer.roll', 'float')]
    period_ms2 = 10
    tello.LOGGER.add_log_config(
        name2,
        variables2,
        period_ms2,
        #   callback=pos_callback
    )
    # Add a log for sensor data
    name3 = 'sen_log'
    variables3 = [tello.LOGGER.LogVariable('range.left', 'float'),
                  tello.LOGGER.LogVariable('range.right', 'float'),
                  tello.LOGGER.LogVariable('range.front', 'float'),
                  tello.LOGGER.LogVariable('range.back', 'float'),
                  tello.LOGGER.LogVariable('range.up', 'float'),
                  tello.LOGGER.LogVariable('range.zrange', 'float')]
    period_ms3 = 10
    tello.LOGGER.add_log_config(
        name3,
        variables3,
        period_ms3,
        #   callback=sen_callback
    )

    # Start flying
    time.sleep(2)   # wait to stabilize
    tello.takeoff()
    tello.send_rc_control(0, 0, 0, 0)
    time.sleep(5)   # wait to stabilize

    # client.set_param('pid_rate.yaw_ki', '1.00')
    # client.set_param('pid_attitude.yaw_kp', '0.00')
    # client.set_param('pid_attitude.yaw_ki', '0.00')
    time.sleep(5)   # wait to orient upwind

    # Main loop, will continue until signaled to land
    while not end_flag and not dist_flag:
        if auto_flag:  # execute cast-and-surge
            #   f.write("starting cast and surge\n")
            #   f.write("starting cast and surge\n")
            #   f.write("starting cast and surge\n")
            # check if obstacles
            if sen[2] < MIN:
                print("*****obstacle ahead at {} m, stopping".format(sen[2]))
                tello.send_rc_control(0, 0, 0, 0)
                # end_flag = True
                state = 1
            elif fan_demo:  # don't care about side obstacles during fan demo
                pass
            elif sen[0] < MIN and (state == 1.5 or state == 3.5):
                print("*****obstacle to left at {} m".format(sen[0]))
                tello.move_left(moveDist)
                state = 2  # switch to going right
            elif sen[1] < MIN and state == 2.5:
                print("*****obstacle to right at {} m".format(sen[1]))
                tello.move_right(moveDist)
                state = 1  # switch to going left

            # check if surge
            if eag_flag and not end_flag:
                print("*****encountered smell")
                if state != 0:  # if doing something else (other than surging), stop
                    tello.send_rc_control(0, 0, 0, 0)
                state = 0  # switch to surging state
                eag_flag = False

            # if action has finished, start new action
            tello.set_speed(1)
            if state*2 % 2 == 0:  # not an in progress state (in progress ends in .5)
                if state == 0:  # start surging
                    if fan_demo:
                        state = -1.5
                    else:
                        state = 0.5  # set state to in progress surging
                    dist = 25  # amount to move forward
                    tello.move_forward(int(dist))
                    print("*****surging now {}m".format(dist))
                elif state == -1:  # hovering
                    pass
                elif state == 1:  # start going left
                    state = 1.5  # set state to in progress going left
                    inc += 6.25
                    dist = 12.5 + inc  # amount to move left
                    tello.move_left(int(dist))
                    print("*****going left now {}m".format(dist))
                elif state == 2:  # start going right
                    state = 2.5  # set state to in progress going right
                    inc += 6.25
                    dist = 12.5 + inc  # amount to move right
                    print("*****going right now {}m".format(dist))
                    tello.move_left(int(dist))
                else:  # start going half left
                    state = 3.5  # set state in progress going half left
                    inc = 0.0625
                    dist = 0.125  # amount to move left
                    print("*****switching to casting, going left now {}m".format(dist))
                    tello.move_left(int(dist))

            tello.send_rc_control(0, 0, 0, 0)
            if state == -1.5:  # ending surge (only happens if in fan_demo mode)
                print("*****state from surge to hover")
                state = -1  # begin hover
            elif state == 1.5:  # end left
                print("*****state from left to right")
                state = 2  # begin right
            elif state == 2.5:  # end right
                print("*****state from right to left")
                state = 1  # begin left
            elif state == 3.5:  # end half left
                print("*****state from half left to right")
                state = 2  # begin right
            else:  # end surging
                print("*****state from surge to half left")
                state = 3  # begin half left motion

        else:  # manual flight controlled via keypress, not doing cast-and-surge
            if vel[2] != 0:
                tello.rotate_counter_clockwise(90)
            elif vel[3] != 0:
                tello.rotate_clockwise(90)
            else:
                tello.send_rc_control(vel[1], 0, 0, 0)
            pass
        #   f.write("time: {}".format(datetime.datetime.now()))
        #   f.write("sensors: {}\n".format(sen))
        #   f.write("pose: {}\n".format(pos))
        #   f.write("eag: {}\n".format(eag_list))
        if len(eag_list) > 0:
            print(max(eag_list))
        eag_list = []
        time.sleep(0.1)  # wait 1/10 of a second, then repeat while loop
    # end while loop

    # print reason for landing
    print("end triggered: {}".format(end_flag))
    print("top sensor triggered: {}".format(dist_flag))
    tello.send_rc_control(0, 0, 0, 0)
    tello.land()

    #   f.close()
    tello.end()
    #   rospy.signal_shutdown(0)
