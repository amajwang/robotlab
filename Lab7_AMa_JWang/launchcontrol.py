
#!/usr/bin/env python
import rospy

import usb.core

# a class to handle communicatinos with the launcher
class LaunchControl():
    def setVSpeed(self, VSpeed):
        """
        Set the up/down speed of the launcher.

        VSpeed should be between -1 and 1, where negative values move the
        launcher down and positive values move it up.

        Note: If abs(VSpeed) + abs(HSpeed) > 1, it will move at the given speed
        horizontally but only 1-abs(HSpeed) vertically. This is because the
        launcher can only either move horizontally or vertically and not both
        at the same time.
        """
        self.VSpeed = VSpeed
        if VSpeed >= 1:
            self.stopPWM()
            self.sendCodeToLauncher('up')
        elif VSpeed <= -1:
            self.stopPWM()
            self.sendCodeToLauncher('down')
        elif VSpeed == 0 and self.HSpeed == 0:
            self.stopPWM()
            self.sendCodeToLauncher('stop')
        else:
            self.ensurePWM()

    def setHSpeed(self, HSpeed):
        """
        Set the left/right speed of the launcher.

        HSpeed should be between -1 and 1, where negative values move the
        launcher left and positive values move it right.

        Note: If abs(VSpeed) + abs(HSpeed) > 1, it will move at the given speed
        horizontally but only 1-abs(HSpeed) vertically. This is because the
        launcher can only either move horizontally or vertically and not both
        at the same time.
        """
        self.HSpeed = HSpeed
        if HSpeed >= 1:
            self.stopPWM()
            self.sendCodeToLauncher('left')
        elif HSpeed <= -1:
            self.stopPWM()
            self.sendCodeToLauncher('right')
        elif HSpeed == 0 and self.VSpeed == 0:
            self.stopPWM()
            self.sendCodeToLauncher('stop')
        else:
            self.ensurePWM()

    def fire(self):
        """
        Stops movement and fires a dart.

        Note that if another method is called right after this one, the launcher
        will not fire.
        """
        self.stopPWM()
        self.sendCodeToLauncher('fire')

    def stop(self):
        """
        Stops movement
        """
        self.stopPWM()
        self.sendCodeToLauncher('stop')

    def setPWMResolution(self, res):
        """
        Set how many different speeds the launcer can move at.

        Higher values may cause jerkines unless the period is increased as well
        
        This defaults to 10
        """
        self.NUM_SPEEDS = res

    def setPWMPeriod(self, period):
        """
        Sets how often (in seconds) the PWM loop runs at.

        Lower values can cause jerkiness, higher values can cause system
        slowdownor other errors (data being sent too fast?)

        This defaults to 0.001 (loop running 1000 times per second)
        """
        self.period = period
   
    # Constructor/Destructor
    def __init__(self):
        self.connectToLauncher();

        self.CODE_UP = [0x02,0x02,0x00,0x00,0x00,0x00,0x00,0x00]
        self.CODE_DOWN = [0x02,0x01,0x00,0x00,0x00,0x00,0x00,0x00]
        self.CODE_LEFT = [0x02,0x04,0x00,0x00,0x00,0x00,0x00,0x00]
        self.CODE_RIGHT = [0x02,0x08,0x00,0x00,0x00,0x00,0x00,0x00]
        self.CODE_STOP = [0x02,0x20,0x00,0x00,0x00,0x00,0x00,0x00]
        self.CODE_FIRE = [0x02,0x10,0x00,0x00,0x00,0x00,0x00,0x00]

        self.callbackNum = 0

        self.HSpeed = 0
        self.VSpeed = 0

        self.setPWMResolution(10)
        self.setPWMPeriod(.001)

    def __del__(self):
        self.stop()

    # Private methods
    def ensurePWM(self):
        """
        Makes sure that the PWM loop is running
        """
        try:
            alive = self.pwmtimer.isAlive()
        except AttributeError:
            alive = False
        if not alive:
            self.newPWM()
            

    def newPWM(self):
        """
        Stops the old PWM loop, if there is one, and starts a new one with
        the given period
        """
        try:
            self.pwmtimer.shutdown()
        except AttributeError:
            pass
        self.pwmtimer = rospy.Timer(rospy.Duration(self.period),
                                    self.control_speed_callback)

    def stopPWM(self):
        """
        Stops the PWM loop
        """
        try:
            self.pwmtimer.shutdown()
        except AttributeError:
            pass

    def connectToLauncher(self):
        """
        A helper method for the constructor that connects to the launcher
        """
        self.launcher = usb.core.find(idVendor=0x2123, idProduct=0x1010)
        if self.launcher is None:
           raise ValueError('Launcher not found.')
        if self.launcher.is_kernel_driver_active(0) is True:
           self.launcher.detach_kernel_driver(0)
        self.launcher.set_configuration()
      
    def sendCodeToLauncher(self, directive):
        """
        Sends a code to the launcher
        """
        if directive == "up": code = self.CODE_UP
        elif directive == "down": code = self.CODE_DOWN
        elif directive == "left": code = self.CODE_LEFT
        elif directive == "right": code = self.CODE_RIGHT
        elif directive == "fire": code = self.CODE_FIRE
        elif directive == "stop": code = self.CODE_STOP
        else:
            print "Error: unrecognized directive in sendCodeToLauncher:", directive
            code = None

        # if everything is OK, send the code to the missile launcher
        if code != None:
            self.launcher.ctrl_transfer(0x21,0x09,0,0,code)

    def control_speed_callback(self, _):
        """
        The function called by the PWM timer that turns the launcher either
        on or off very fast

        The way this method works is by dividing up time into NUM_SPEEDS
        periods. The first abs(HSpeed) proportion of these are devoted to
        sending the launcher either left or right; after that, the next
        abs(VSpeed) proportion of these are devoted to sending the launcher
        either up or down; and finally movement is stopped.

        Possible improvements:
        * Instead of doing all of the horizontal movement, then all of the
          vertical movement, then stop, it would be smoother if they were
          interleaved, if possible. This would let us reduce the period.
        """

        curProp = float(self.callbackNum)/self.NUM_SPEEDS

        # Subtract a little so that we round to nearest rather than round down
        if curProp < abs(self.HSpeed) - .5/self.NUM_SPEEDS:
            self.sendCodeToLauncher('left' if self.HSpeed > 0 else 'right')

        elif curProp < abs(self.HSpeed) + abs(self.VSpeed) - .5/self.NUM_SPEEDS:
            self.sendCodeToLauncher('up' if self.VSpeed > 0 else 'down')

        else:
            self.sendCodeToLauncher('stop')

        # Keep track of our state so we know if we should be moving or not
        self.callbackNum += 1
        if self.callbackNum >= self.NUM_SPEEDS:
            self.callbackNum = 0


# If not being imported, run the test program
if __name__ == '__main__':
    # Initialize our node
    rospy.init_node('launcher_controller_tester')

    lc = LaunchControl()

    print "Gives space separated speeds, or 'q' (quit), 'fire', or 'stop'"
    while True:
        ln = raw_input('Speeds (x y): ')
        if ln == 'q':
            break
        elif ln == 'fire':
            lc.fire()
            continue
        elif ln == 'stop':
            lc.stop()
            continue
        x, y = [float(s) for s in ln.split()]
        lc.setHSpeed(x)
        lc.setVSpeed(y)

