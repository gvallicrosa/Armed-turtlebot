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

    def callback(self, msg):
        """Callback"""
        self.atTarget = msg

    def task(self, statusServices=[]):
        """Approach the target."""

        # Are we at the target?
        # The camera must publish this topic.
        rospy.Subscriber('camera_atTarget', Int8, self.callback)
        
        if(self.atTarget):
            # If we are at the target then update belief.
            rospy.loginfo('gotoTarget: Arrived at target.')
            self.requestService(mc_updateBelief, ("atTarget", 1))
        else:
            # Else move towards target.
            self.requestService(motionControl_move, (-0.1, 0.0))
            rospy.loginfo('gotoTarget: moving forwards.')
