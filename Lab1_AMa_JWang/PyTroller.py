# PyTriller.py
# joystick reader and parser
# by Andrew Fishberg et al. Spring 2014

import sys
import roslib; roslib.load_manifest('irobot_mudd')
import rospy
import sensor_msgs.msg as sm
from std_msgs.msg import String
from ControllerParser import update_D

class Data: pass
D = Data()
D.BTNS = [0]*52
D.PADS = [0]*52


def prettyPrintD():
  global D
  #for i, val in enumerate(D.BTNS):
  #  print '{0:2d} {1:3d}'.format(i, val)
  print "BTNS:", D.BTNS
  print "PADS:", D.PADS

def toHexString(hex):
  """ handles some data formatting details """
  return "0x" + "".join(reversed(hex[4:]))


def main():
    """ the main function that sets everything up and runs...
    """
    global D
    # Initialize this ROS node
    rospy.init_node('controller_publisher') # any name will do

    # open the joystick and read its data
    # I have not idea what action is for, but I'm too afraid to remove it...
    pipe = open('/dev/input/js0', 'r')
    action = []

    pub = rospy.Publisher('controller_data', String)

    # main loop
    # here, a person can type messages, which will be published
    while rospy.is_shutdown() == False: # standard infinite loop in ROS
       #print "Loop"
       for character in pipe.read(1):
          #print "Loop"
          action += ['%02X' % ord(character)]
          if len(action) == 8:
            hstr = toHexString(action)
            update_D( D, hstr )
            prettyPrintD()
            print "Publishing ", hstr
            pub.publish(String(hstr)) # publish!

            #
            # publish! (or perish...)
            #
            # here, you COULD publish hstr and parse it elsewhere
            #print "Publishing", hstr
            #pub.publish(String(hstr))
            
            # or, you could check the value of D.BTNS and D.PADS
            # and send only the messages you'd like... up to you!
            # ... in this case, you'd publish a string of your
            # own design...
            # s = ...
            #pub.publish(String(s))

            # I have no idea what this is...
            action = []

####
# It all starts here...
#
# This is the "main" trick: it tells Python what code to run
# when you execute this file as a stand-alone script:
####

if __name__ == "__main__":
    main()






# some commented code from the solution (feel free to use or re-derive)
    #pub = rospy.Publisher('controller_data', String)
