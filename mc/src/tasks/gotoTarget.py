#!/usr/bin/env python

# Python modules

# ROS modules
import roslib
#roslib.load_manifest('mc')
import rospy
from std_msgs.msg import Float64, String

# Custom modules
from task import task
from mc.srv import mc_updateBelief
from mc.srv import motionControl_move

###############################################################################

class gotoTarget(task):

    name = "gotoTarget"

    def __init__(self):
        # Assume we are not at the target.
        self.atTarget = 0
        self.measuredRadius = 0.0
        self.physicalRadius = 2.16 # Actual ball radius in cm.

    def updateRadiusHandler(self, msg):
        """Callback function for subscribes."""
        self.measuredRadius = float(msg.data.split(' ')[2])

    def task(self, statusServices=[]):
        """Approach the target."""

        # Loop until we are at the target.
        while(self.atTarget == 0):

            # What is the apparent radius of the ball?
            # (Published by detection node)
            rospy.Subscriber('/detection/posrad', String, self.updateRadiusHandler)
            rospy.logdebug("gotoTarget: Measured target radius = " + str(self.measuredRadius))

            ###################################################################

            # Are we at the target yet?
            if(self.measuredRadius > 10.0): # <-----|Value to be determined by experiment.|
                self.atTarget = 1
                rospy.logdebug("gotoTarget: Arrived at target (measured target radius = " + str(self.measuredRadius) + ").")

            ###################################################################

            # If not at target then move forwards.
            if(self.atTarget == 0):
                self.requestService(motionControl_move, (-0.1, 0.0))
                rospy.logdebug('gotoTarget: moving forwards.')

        # Update belief on MC
        self.requestService(mc_updateBelief, ("atTarget", 1))
