#!/usr/bin/python
#############################################
##    Original authors: Cyrus Huang, Zakkai Davidson
##    Freed from the Neato and rewritten by Adam Dunlap!
##    NOTE:
##    Run laser driver first with
##
##    sudo chmod 777 /dev/ttyUSB* ; rosrun neato_lidar driver.py
#############################################
import roslib; roslib.load_manifest('neato_lidar')
import rospy
import cv
import time
import neato_lidar
import neato_lidar.msg

from std_msgs.msg import *
from sensor_msgs.msg import *
from math import *
from cmath import polar

# class for a generic data holder
class Data:  pass
D = Data()         # our global data object, D

# window variables
WIN_WIDTH  = 600                # keeps square windows
WIN_HEIGHT = WIN_WIDTH
SIZE       = (WIN_WIDTH,WIN_HEIGHT)
CENTER     = WIN_WIDTH/2


# line drawing variables
MAX_MAG      = 1000             # unreliable data above 10m
MAG_SCALE    = 100              # 100 pixels per meter
ANGLE_OFFSET = -90              # front of robot faces up on screen
REV          = 360              # 360 degrees per rev (range data is stored in degrees)

# options
SHOW_HOUGH = False              # set to true to show Hough transformation image



def init_GUI():
    """ initializes open cv windows and creates images to display """
    global D, pub, pub_laser

    print
    print "Press 'q' in Ranges window to quit"
    
    # create window and image to show range values
    cv.NamedWindow("Ranges")
    cv.MoveWindow("Ranges", WIN_WIDTH/2, WIN_HEIGHT/2)
    D.image = cv.CreateImage(SIZE, 8, 3) # 8 is pixel depth and 3 is # of channels

    # window for Hough transformation
    if SHOW_HOUGH:
        cv.NamedWindow("HoughInput")
        cv.MoveWindow("HoughInput", WIN_WIDTH/2 + WIN_WIDTH, WIN_HEIGHT/2)
        
    # image for Hough transformation
    D.hough = cv.CreateImage((WIN_WIDTH,WIN_HEIGHT), 8, 1) # image used for Hough transformation

    # initialize ROS subscription
    rospy.init_node("range_listener", anonymous=True)

    # subscribe to laser data
    rospy.Subscriber('lidar', neato_lidar.msg.LidarRanges, range_callback)
    
    # storage needed for Hough processing
    D.storage = cv.CreateMemStorage(0)
    
    # give initial values to range data before first callback
    D.ranges =[0]*REV


    # create a String publisher (pub)
    pub = rospy.Publisher('keyboard_data',String)

    pub_laser = rospy.Publisher('laser_data',String)



def range_callback(data):
    """ callback to handle ranges """
    global D
    D.ranges = data.ranges



def handle_key_presses():
    """ does just that! """
    global D

    # wait and check for quit request
    key_code = cv.WaitKey(1) & 255    # numeric value is key_code
    key_press = chr(key_code)         # string vaue is key_press

    # check for ESC or 'q'
    if key_code == 27 or key_press == 'q' : # if ESC or 'q' was pressed
        time.sleep(0.1)  # pause for a moment...    # then, shutdown:
        pub.publish(String('q')) # publish the string 'q' either way
        rospy.signal_shutdown( "Quitting..." )

    if ' ' <= key_press <= 'z': # if it's in our valid range
        print "Publishing ", str(key_press)
        pub.publish(String(str(key_press))) # publish!

    return


def draw_laser_ranges():
    """ this function should
          (1) draw to the D.image window the ranges as rays
              the walls (once computed), and
              their midpoints
          (2) it should also draw to the D.hough image the dots
              needed as input to the Hough transformation
    """

    scale = 0.2

    if True:  # for easy commenting out...
        NUM_RANGES = len(D.ranges) # should be 360
        for angle in range(NUM_RANGES):
            # print angle, ":", D.ranges[angle]
            r, theta = scale*D.ranges[angle], radians(angle+84)
            end_x, end_y = CENTER + int(r*cos(theta)), CENTER - int(r*sin(theta))
            # add line to the ranges image, "D.image"
            cv.Line(D.image, (CENTER,CENTER), (end_x,end_y), cv.RGB(255, 0, 0), 1) # 1 == thickness
            # add dots to image being used to compute the Hough tr. "D.hough"
            cv.Line(D.hough, (end_x,end_y), (end_x,end_y), 255, 2) # 1 == thickness
     
    return


def findMPW(Lines):
    """ 
    """
    MPW_left = MPW_right = MPW_front = MPW_default = [((0,0),(0,0)),0]

    def getLineInfo(line):

        x0, y0, x1, y1 = line[0][0]-CENTER, line[0][1]-CENTER, line[1][0]-CENTER, line[1][1]-CENTER
        A, B, C = (y1-y0), -(x1-x0), (x1*y0-x0*y1) # Ax + By + C = 0

        length = sqrt(A**2 + B**2)
        xi,yi = -A*C/length**2, -B*C/length**2
        r,theta = polar(1j*xi-yi)

        return [line, theta, r, length]

    lineinfos = [getLineInfo(line) for line in Lines]


    def getFilter(theta, threshold = pi/4):
        return lambda x: abs(x[1]-theta) < threshold

    def getCompareKey(theta):
        return lambda x: abs(x[1]-theta)

    def getMPW(theta):
        filtered = filter(getFilter(theta), lineinfos)
        return min(filtered, key = getCompareKey(theta)) if filtered else []

    # print filter(getFilter(-pi/2), lineinfos)

    MPW_left = getMPW(-pi/2)
    MPW_right = getMPW(pi/2)
    MPW_front = getMPW(0)

    # if MPW_left == MPW_default or abs(theta+pi/2) < abs(MPW_left[1]+pi/2):
    #         MPW_left = [line, theta]

    # if MPW_right == MPW_default or abs(theta-pi/2) < abs(MPW_right[1]-pi/2):
    #         MPW_right = [line, theta]

    # if MPW_front == MPW_default or abs(theta) < abs(MPW_front[1]):
    #         MPW_front = [line, theta]

    if MPW_left:
        cv.Line(D.image, *MPW_left[0], color = cv.RGB(0, 255, 255), thickness = 5) # cyan

    if MPW_right:
        cv.Line(D.image, *MPW_right[0], color = cv.RGB(255, 0, 255), thickness = 5) # magenta

    if MPW_front:
        cv.Line(D.image, *MPW_front[0], color = cv.RGB(255, 255, 0), thickness = 5) # yellow 

    pub_laser.publish(String(str([MPW_left, MPW_right, MPW_front])))



def findHoughLines():
    """ Uses the Hough transformation to find lines from the sensor
        readings and displays them
    """
    global D

    # apply Hough transformation to find straight line segments
    # For more information, see:
    # http://docs.opencv.org/doc/tutorials/imgproc/imgtrans/hough_lines/hough_lines.html
    #
    # Here are the  options for the Hough transformation

    # distance_res = 3  # distance resolution of Hough accumulator, in pixels
    # angle_res    = radians(10)  # angular resolution of Hough acc., in radians
    # min_votes    = 20 # how many votes needed to count a line?
    # min_line_len = 5 # shortest allowable line, in pixels
    # max_gap_len  = 30 # pixels
    
    distance_res = 1  # distance resolution of Hough accumulator, in pixels
    angle_res    = radians(1)  # angular resolution of Hough acc., in radians
    min_votes    = 20 # how many votes needed to count a line?
    min_line_len = 30 # shortest allowable line, in pixels
    max_gap_len  = 30 # pixels
    
    # The return value is a list of line segments
    Lines = cv.HoughLines2(D.hough,
                           D.storage,
                           cv.CV_HOUGH_PROBABILISTIC,
                           distance_res, 
                           angle_res,
                           min_votes,
                           min_line_len,
                           max_gap_len)

    N = len(Lines)
    # print "\n\n"
    for i in range(N):
        line = Lines[i]
        # print "line[",i,"] is", line
        cv.Line(D.image, *line, color = cv.RGB(0, 255, 0), thickness = 1)

    findMPW(Lines)



def main():
    """ creates and displays a GUI for the range finder data
        Ranges window: shows range finder values as red lines
        coming from the center of the range finder
        HoughLines window: shows the result of using a Hough
        transformation on image containing the range values as points.
    """
    global D
    
    init_GUI() # initialize images and windows


    # main loop
    while rospy.is_shutdown() == False:

        # handle any pending keypresses... - need to keep this
        handle_key_presses()
        
        # draw the ranges in D.ranges to the screen
        draw_laser_ranges()

        # find walls and add to image using Hough transformation
        findHoughLines()

        # show color image with range finder data and calculated walls
        cv.ShowImage("Ranges",  D.image)
        
        # show b/w image used as the input to the Hough transformation, if desired
        if SHOW_HOUGH:  cv.ShowImage("HoughInput", D.hough) 

        # clear the images for next loop
        cv.Set(D.image, cv.RGB(0, 0, 0))
        cv.Set(D.hough, cv.RGB(0, 0, 0))


    print "Quitting..."
            
     

    
if __name__ == "__main__":
    main()