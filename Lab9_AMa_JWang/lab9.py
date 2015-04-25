#!/usr/bin/env python
import roslib; roslib.load_manifest('irobot_mudd')
import rospy
import sensor_msgs.msg as sm
import cv_bridge
import cv
# import numpy
# import math
from math import *

# class for a generic data holder
class Data:
    def __init__(self): pass    # empty constructor
    
# object (of class Data) to hold the global state of the system
D = Data()


def startup():
    """Sets up things needed for the class"""
    #Create cv stuff
    cv.NamedWindow('dist')
    D.bridge = cv_bridge.CvBridge()
    cv.SetMouseCallback('dist', onMouse, None)
    D.color_image = None

    D.scale = 1
    D.left_x = 250
    D.right_x = 390
    D.vertical = 240
    D.up_y = 220
    D.down_y = 260
    D.horizontal = 320

    make_slider_window()
    D.font = cv.InitFont(cv.CV_FONT_HERSHEY_SIMPLEX, .5, .5, 0, 1)

def make_slider(value, maximum):
    cv.CreateTrackbar(value, 'sliders', D.left_x, maximum, change_value(value))

def make_slider_window():
        """ a method to make a window full of sliders """
        #Create slider window
        cv.NamedWindow('sliders')
        cv.MoveWindow('sliders', 800, 0)

        #Create sliders
        sliderlist = [('left_x',640), ('right_x',640), ('vertical',480), ('up_y',480), ('down_y',480), ('horizontal',640)]
        for s in sliderlist:
            make_slider(*s)

        cv.CreateTrackbar('scale', 'sliders', D.scale, 10, change_scale)

#Functions for changing the slider values  
def set_global(value, newval):
    exec("D." + value + " = " + str(newval))

def change_value(value):
    return lambda newval : set_global(value, newval)

def change_scale(newval):
    D.scale = newval
    if D.scale < 1: D.scale = 1


def get_coordinates(pixel_x, pixel_y):
    depth = D.image[pixel_y, pixel_x]
    x = depth * tan(radians(28.5)) * (pixel_x-320) / 320
    y = depth
    z = depth * tan(radians(21.5)) * (pixel_y-240) / 240
    return (x,y,z)
        
def handle_next_image(data):
    """Displays the image, calls find_info"""        
    # get the image from the Kinect
    D.image = D.bridge.imgmsg_to_cv(data, "32FC1")
    cv.Flip( D.image, D.image, 1 )  # flip it!

    # change the scale
    cv.ConvertScale( D.image, D.image, scale=1.0/D.scale, shift=0.0 )

    # create a new color image, if it does not yet exist
    if D.color_image == None:
        D.color_image = cv.CreateImage(cv.GetSize(D.image),cv.IPL_DEPTH_32F, 3)

    # create *this frame's* color image, now that we know one exists
    cv.Merge( D.image, D.image, D.image, None, D.color_image )

    # handle key presses
    key_press = cv.WaitKey(5) & 255
    if key_press != 255: check_key_press( key_press )

    # computation and other processing can go here!
    cv.Circle(D.color_image, (D.left_x, D.vertical),
              8,  cv.RGB(255,0,0),  thickness=1, lineType=8, shift=0)

    cv.Circle(D.color_image, (D.right_x, D.vertical),
              8,  cv.RGB(255,0,0),  thickness=1, lineType=8, shift=0)

    x1, y1, z1 = get_coordinates(D.left_x, D.vertical)
    x2, y2, z2 = get_coordinates(D.right_x, D.vertical)

    pan = degrees(atan2(y2-y1, x2-x1))


    cv.Circle(D.color_image, (D.horizontal, D.up_y),
              8,  cv.RGB(255,0,0),  thickness=1, lineType=8, shift=0)

    cv.Circle(D.color_image, (D.horizontal, D.down_y),
              8,  cv.RGB(255,0,0),  thickness=1, lineType=8, shift=0)

    x3, y3, z3 = get_coordinates(D.horizontal, D.up_y)
    x4, y4, z4 = get_coordinates(D.horizontal, D.down_y)

    tilt = degrees(atan2(y4-y3, z4-z3))
    print pan, tilt


    # show text...
    draw_text_to_image()

    # display the image
    cv.ShowImage('dist', D.color_image)


def onMouse(event,x,y,flags,param):
    """ the method called when the mouse is clicked """
    # if the left button was clicked
    if event==cv.CV_EVENT_LBUTTONDOWN: 
        print "x, y are", x, y,
        pixel_value = D.image[y,x]
        print "the pixel's depth value is", pixel_value


# key-press handler
def check_key_press(key_press):
    """ this method handles user key presses appropriately """
    # if a 'q' or Esc was pressed
    if key_press == 27 or key_press == ord('q'): 
        print 'quitting'
        rospy.signal_shutdown( "Quit requested from keyboard" )


def draw_text_to_image():
    """ A function to handle drawing things to the image """
    # draw a rectangle under the text to make it more visible
    cv.Rectangle(D.color_image, (25,25), (100,50), cv.RGB(0,.42,0), cv.CV_FILLED)
    # place some text there
    cv.PutText(D.color_image, "Hi!", (30,40), D.font, cv.RGB(1,1,1))

        

if __name__ == "__main__":
    """Main function, sets up stuff the class needs to run and runs it"""
    
    #Initialize our node
    rospy.init_node('distanceReader')
    
    #Create a SegmentFinder
    startup()
    
    #Subscribe to the image topic
    rospy.Subscriber('/camera/depth/image',sm.Image,handle_next_image)
    
    #Run until something stops us
    rospy.spin()

    