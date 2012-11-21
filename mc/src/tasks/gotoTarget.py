#!/usr/bin/env python

# Python modules

# ROS modules
import roslib
#roslib.load_manifest('mc')
import rospy
from std_msgs.msg import Int8

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
        self.measuredRadius = msg 

    def task(self, statusServices=[]):
        """Approach the target."""

        # Loop until we are at the target.
        while(self.atTarget == 0):

            # What is the apparent radius of the ball?
            # (Published by detection node)
            rospy.Subscriber('/detection/radius', Float32, self.updateRadiusHandler)

            ###################################################################

            # Are we at the target yet?
            if(self.measuredRadius == 10): # <-----|Value to be determined by experiment.|
                self.atTarget = 1

            ###################################################################

            # If not at target then move forwards.
            if(self.atTarget == 0):
                self.requestService(motionControl_move, (-0.1, 0.0))
                rospy.loginfo('gotoTarget: moving forwards.')

        # We are at the target - update belief.
        rospy.loginfo('gotoTarget: Arrived at target.')
        self.requestService(mc_updateBelief, ("atTarget", 1))
            
