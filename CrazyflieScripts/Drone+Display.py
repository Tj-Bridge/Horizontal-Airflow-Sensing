from time import sleep
from djitellopy import Tello
import numpy
import logging
import matplotlib.pyplot as plt
import serial
from matplotlib.offsetbox import AnnotationBbox, OffsetImage
from PIL import Image
import matplotlib.patches as mpatches

# statements for Arduino serial communication
logging.basicConfig(level=logging.ERROR)
arduino = serial.Serial(port='COM3', baudrate=9600, timeout=.05)

# random initial values to start loop
angle_diff = 16
last_angleAirflow = 0
fig = plt.figure()
ax = fig.add_subplot(1, 1, 1)

'''
# connecting to tello
tello = Tello()
tello.connect()

# returns battery percentage and makes drone takeoff
tello.query_battery()
tello.takeoff()
'''
while True:
    # Move left y-axis and bottim x-axis to centre, passing through (0,0)
    ax.spines['left'].set_position('center')
    ax.spines['bottom'].set_position('center')

    # Eliminate upper and right axes
    ax.spines['right'].set_color('none')
    ax.spines['top'].set_color('none')

    # Show ticks in the left and lower axes only
    ax.xaxis.set_ticks_position('bottom')
    ax.yaxis.set_ticks_position('left')
    ax.set_xlim(-180, 180)
    ax.set_ylim(-180, 180)

    red_patch = mpatches.Patch(color='red', label='Flight Direction')
    blue_patch = mpatches.Patch(color='blue', label='Airflow Direction')

    ax.legend(handles=[red_patch, blue_patch])

    origin = numpy.array([[0, 0, 0], [0, 0, 0]])  # origin point
    data = []  # empty list to store the data
    for i in range(50):
        b = arduino.readline()  # read a byte string
        string_n = b.decode()  # decode byte string into Unicode
        string_n1 = string_n.rstrip()
        string_n2 = string_n1.split(",")
    # if array has contents, run the code
    if len(string_n2) > 1:
        # maps array objects to float type and appends to list that can be addressed
        map_object = map(float, string_n2)
        #print("good")
        value_list = list(map_object)
        #print(value_list)
        y = value_list[1]
        y = y * -1
        #print(x)
        x = value_list[0]*-1
        x = x * -1
        data.append(string_n2)  # add to the end of data list

        y2 = value_list[1]
        x2 = value_list[0]*-1
        V = numpy.array([[y, x]])
        V2 = numpy.array([[y2, x2]])
        #plt.xlim(-1000, 1000)
        #plt.ylim(-1000, 1000)

        drone = Image.open('drone-pic.png').convert('RGBA')  # insert local path of the image.
        imagebox_python = OffsetImage(drone)
        xy = [-15, 20]
        #ab_drone = AnnotationBbox(imagebox_python, xy, xybox=(10., -10.), boxcoords='offset points', frameon=False)
        #ax.add_artist(ab_drone)
        plt.quiver(*origin, V[:, 0], V[:, 1], color=['b'], scale=3000, zorder=10)
        plt.quiver(*origin, V2[:, 0], V2[:, 1], color=['r'], scale=3000, zorder=10)


        # takes x and y values and makes angle
        angleAirflow = int(numpy.arctan2(y, x) * 180 / numpy.pi)
        #print('Angle:', angleAirflow)
        # difference in angles to get relative angle
        angle_diff = angleAirflow - last_angleAirflow
        print("angle_diff:", angle_diff)

        # logic to turn
        if abs(angle_diff) > 20:
            angleDrone = angle_diff
            if angle_diff < 0:
                print("turning left")
            elif angle_diff > 0:
                print("turning right")
            #tello.rotate_counter_clockwise(angleDrone)
            last_angleAirflow = angleAirflow
        elif angle_diff == 0:
            #tello.send_control_command("stop",timeout=10)
            print("hovering")

    else:
        print("something's wrong with data")
        #tello.query_battery()

    plt.pause(0.005)
    plt.cla()
mng.full_screen_toggle()
mng = plt.get_current_fig_manager()

plt.show()
serial.close()
#tello.query_battery()








'''
            if angle_diff > 15:
                if angleAirflow <= 180 and angleAirflow > 0:
                    angleDrone = angleAirflow + 180
                    tello.rotate_counter_clockwise(angleDrone)
                    print("turning left")
                # drone turns leftwards
                elif angleAirflow == 0:
                    angleDrone = 1
                    print("chillin")
                elif angleAirflow < 0:
                    angleDrone = angleAirflow - 180
                    tello.rotate_counter_clockwise(angleDrone)
                    print("turning right")
                last_angleAirflow = angleAirflow
'''


