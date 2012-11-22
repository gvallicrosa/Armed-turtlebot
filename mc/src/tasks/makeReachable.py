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

class makeReachable(task):

    name = "makeReachable"

    def __init__(self):
        # Assume we cannot reach the target.
        self.targetReachable = 0

    def callback(self, msg):
        """Callback"""
        self.targetReachable = msg

    def task(self, statusServices=[]):
        """Approach the target."""

        # Is the target reachable?
        # Provided by samrt arm node.
        rospy.Subscriber('vs_targetReachable', Int8, self.callback)

        if(self.targetReachable):
            # If target is reachable then update belief.
            rospy.loginfo('makeReachable: target should be reachable from here.')
            self.requestService(mc_updateBelief, ("targetReachable", 1))
        else:
            # Else move towards target.
            self.requestService(motionControl_move, (-0.1, 0.0))
            rospy.loginfo('gotoTarget: moving forwards.')
