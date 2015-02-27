
#
# brains0.py
#

import roslib; roslib.load_manifest('irobot_mudd')  # irobot_mudd
import rospy
import cv
import time
from std_msgs.msg import *
from sensor_msgs.msg import *
from irobot_mudd.srv import *
from irobot_mudd.msg import *
from math import *


# class for a generic data holder
class Data:  pass
D = Data()             # our global data object, D

D.STATE = "WAITING_TO_START"
# States!
"WAITING_TO_START"               # the very first state entered
"ROTATE_IN_PLACE"                # tries to get parallel to the MPW
"BACKING_UP_FOR_A_BIT"           # if we bump...


# globals for incoming data streams...
D.list_laser_data = None
D.start_time = time.time() # start of program
D.last_time_printed = 0    # we'll print once in a while
D.last_time_clocked = D.start_time # the last time we hit the stopwatch



def laser_data_callback(data):
    """ runs with each laser_data message from the laser-handling code """
    global D
    message = data.data # that's the string
    D.list_laser_data = eval( message ) # Yay, Python!
    # print message
    # don't do lots of work here... instead, we'll do it all in main
    # we won't print it... if you want to see this stream use (in a new tab)
    # rostopic list
    # rostopic echo laser_data

def key_press_callback(data):
    """ runs with each message from the keyboard_data stream """
    global D
    message = data.data # that's the string
    D.last_keypress = message
    # we'll handle stuff here...
    k = D.last_keypress

    if k in ' ': 
        D.robot_publisher.publish( "toggle commands" ) # Wow!

    if k in '`': # goes to the waiting state
        D.robot_publisher.publish( "D.tank(0,0)" ) # Yay, Python!
        D.STATE = "WAITING_TO_START"  # back to waiting to start

    if k in 'W':  
        D.STATE = "MOVING_FORWARD"

    if k in 'A':  
        D.STATE = "ROTATE_IN_PLACE_LEFT"

    if k in 'D':  
        D.STATE = "ROTATE_IN_PLACE_RIGHT"

    if k in 'S':  
        D.last_time_clocked = time.time()
        D.robot_publisher.publish( "D.tank(-100,-100)" ) # Yay, Python!
        D.STATE = "BACKING_UP_FOR_A_BIT"


def robot_sensor_callback( data ):
    """ sensor_callback is called for each sensorPacket message
    """
    global D
    if data.wheeldropCaster == True:
        # enable this later!
        #D.robot_publisher.publish( "D.tank(0,0)" ) # Yay, Python!
        #D.STATE = "WAITING_TO_START"  # back to waiting to start
        pass

    if data.advance == True:  
        D.robot_publisher.publish( "D.tank(0,0)" ) # Yay, Python!
        D.STATE = "WAITING_TO_START"  # back to waiting to start
        
    if data.play == True:
        D.robot_publisher.publish( "song()" ) # Yay, Python!
        # Start your state machine here, perhaps!

    if data.bumpRight or data.bumpLeft == True:
        D.last_time_clocked = time.time()
        D.robot_publisher.publish( "D.tank(-100,-100)" ) # Yay, Python!
        D.STATE = "BACKING_UP_FOR_A_BIT"
    # again, we won't print it, but you can use
    # rostopic list
    # rostopic echo sensor


def clock( current_time ):
    """ print once in a while """
    global D
    number_of_seconds_since_start = int(current_time - D.start_time)
    if D.last_time_printed < number_of_seconds_since_start:
        print "[Brains] [State:", D.STATE, "]  time is", \
              number_of_seconds_since_start, "seconds since starting..."
        D.last_time_printed = number_of_seconds_since_start



def main():
    """ creates and displays a GUI for the range finder data
        Ranges window: shows range finder values as red lines
        coming from the center of the range finder
        HoughLines window: shows the result of using a Hough
        transformation on image containing the range values as points.
    """
    global D

    # initialize ROS subscription
    rospy.init_node("robot_brains", anonymous=True)

    # subscribers
    rospy.Subscriber( 'laser_data', String, laser_data_callback)
    rospy.Subscriber( 'sensorPacket', SensorPacket, robot_sensor_callback )
    rospy.Subscriber( 'keyboard_data', String, key_press_callback )

    # publishers
    D.robot_publisher = rospy.Publisher('robot_commands', String)
    D.state_publisher = rospy.Publisher('state_strings', String)

    # 
    # main loop - and state machine!
    #
    while rospy.is_shutdown() == False:
        time.sleep(0.04)  # a little pause for each loop...
        current_time = time.time() ; clock( current_time ) # print every second

        D.state_publisher.publish( D.STATE ) # in another tab: rostopic echo state_strings
        

        # here is our state machine!
        if D.STATE == "BACKING_UP_FOR_A_BIT":
            if current_time > 4.2 + D.last_time_clocked: # 4.2 seconds of wait
                # don't overflow the robot's command buffer...
                D.robot_publisher.publish( "D.tank(0,0)" )
                D.STATE = "ROTATE_IN_PLACE"
            # else keep waiting

        elif D.STATE == "ROTATE_IN_PLACE":
            # use D.list_laser_data to find the amount still needed to rotate...
            # then determine a rotational velocity
            # or, if it's close enough, switch to a forward-going state!
            D.robot_publisher.publish( "D.tank(-50,50)" ) 

        elif D.STATE == "WAITING_TO_START":
            pass # do nothing

        elif D.STATE == "MOVING_FORWARD":
            D.robot_publisher.publish( "D.tank(100,100)" )

        elif D.STATE == "ROTATE_IN_PLACE_LEFT":
            D.robot_publisher.publish( "D.tank(-50,50)" ) 

        elif D.STATE == "ROTATE_IN_PLACE_RIGHT":
            D.robot_publisher.publish( "D.tank(50,-50)" )




        else:
            print "I don't recognize the state", D.STATE


    print "Quitting the main loop..."
            
     
    
if __name__ == "__main__":
    main()