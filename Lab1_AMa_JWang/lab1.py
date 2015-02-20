#!/usr/bin/env python
import roslib; roslib.load_manifest('irobot_mudd')
import rospy
import irobot_mudd
#from ControllerParser import update_D # don't worry about this until part 2
from std_msgs.msg import String
from irobot_mudd.srv import *
from irobot_mudd.msg import *
import time
import math
from ControllerParser import update_D

####
# lab1.py ~ starter file for scripting the Create with ROS
####


####
# D is our global system state
####

class Data: pass    # empty class for a generic data holder
D = Data()  # an object to hold our system's services and state

# some global lists you can ignore until part 2
D.BTNS = [0]*52
D.PADS = [0]*52

####
# square and polygon
####

def polygon(n, d):
    """ let the robot traverse an n-gon
    """
    global D

    time_per_round = 16.8
    time_forward = 2 * d
    time_sleep = 0.4

    for i in range(n):
        
        D.tank(200,200)
        time.sleep(time_forward)

        D.tank(0,0)
        time.sleep(time_sleep)

        D.tank(50,-50)
        time.sleep(time_per_round / n)

        D.tank(0,0)
        time.sleep(time_sleep)


####
# callback ~ called with each published message
####

def callback(data):
    """ This function is called for each published message
    """
    global D

    message = data.data
    print "I received the string", message
    
    # if the message is the string 'q', we shutdown
    if message == 'q':
        rospy.signal_shutdown("Quit requested.")

    if message == 'R':
        D.tank(200,200)
    if message == 'T':
        D.tank(-200,-200)
    if message == 'Q':
        D.tank(-50,50)
    if message == 'S':
        D.tank(50,-50)        
    if message == ' ':
        D.tank(0,0)

    if message == 's':
        D.song([30,55,60,62,63,65,62,63,60],       # notes
               [ 8, 8, 8, 8,24, 8,24, 8,24])       # durations
        time.sleep(.1)
        D.song([60,59,60,62,63,62,63,65,67,67,67], # notes
               [ 8, 8, 8, 8, 8, 8, 8, 8,16,16,24]) # durations

    if message in "3456": #int(message) in [3,4,5,6]:
        polygon(int(message),2)

####
# main and init
####

def main():
    """ the main program that gives our node a name,
       sets up service objects, subscribes to topics (with a callback),
       and then lets interactions happen!
    """
    global D

    # set up services and subscribe to data streams
    init()

    D.leds(0,0,20,100)

    # sing!
    # D.song([30,55,60,62,63,65,62,63,60],       # notes
    #        [ 8, 8, 8, 8,24, 8,24, 8,24])       # durations
    # time.sleep(.1)
    # D.song([60,59,60,62,63,62,63,65,67,67,67], # notes
    #        [ 8, 8, 8, 8, 8, 8, 8, 8,16,16,24]) # durations   

    # move!
    # D.tank(100,100)
    # time.sleep(2.0)
    
    # reminder of Python's for loop:
    for i in range(2):
        print "i is", i

    # polygon(4,2)

    # finish up...
    D.tank(0,0)
    D.leds(0,0,0,0)

    rospy.spin()

    print "Goodbye!"


def controller_callback(data):
    global D

    update_D( D, data.data)
    base_speed = -200;
    D.tank(D.PADS[1]*base_speed,D.PADS[3]*base_speed);




def init():
    """ returns an object (tank) that allows you
       to set the velocities of the robot's wheels
    """
    global D # to hold system state

    # we need to give our program a ROS node name
    # the name is not important, so we use "lab1_node"
    rospy.init_node('lab1_node', anonymous=True)
    
    # we obtain the tank service
    rospy.wait_for_service('tank') # wait until the motors are available
    D.tank = rospy.ServiceProxy('tank', Tank) # D.tank is our "driver"
    
    # we obtain the song service
    rospy.wait_for_service('song') # wait until our voice is available
    D.song = rospy.ServiceProxy('song', Song) # D.song is our "speaker" 

    # we obtain the leds service
    rospy.wait_for_service('leds') # wait until the leds are available
    D.leds = rospy.ServiceProxy('leds', Leds) # D.leds is our "lights"

    # the key piece of information is the name of the published stream
    stream_name = 'text_data'

    # this subscribes to the stream
    # (1) it names the stream (stream_name)
    # (2) it indicates the type of message received (String)
    # (3) it indicates a function (callback) to be called w/ each message
    rospy.Subscriber( stream_name, String, callback )

    rospy.Subscriber( 'controller_data', String, controller_callback )

####
# It all starts here...
#
# This is the "main" trick: it tells Python what code to run
# when you execute this file as a stand-alone script:
#### 

if __name__ == "__main__":
   main()

