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

class locateTarget(task):

    name = "locateTarget"

    def __init__(self):
        # Assume that we can't see the target.
        self.located = 0

    ###########################################################################

    def targetLocated(self, msg):
        self.located = msg

    ###########################################################################

    def task(self, statusServices=[]):
        """Locate the target."""

        # Loop until the target is located.
        while(self.located == 0):

            # Listen if the target has been located. (y/n)
            # This topic must be published by the camera node.
            rospy.Subscriber('camera_targetLocated', Int8, self.targetLocated)

            # Rotate the turtlebot
            self.requestService(motionControl_move, (0.0, 0.1))

        # Stop the turtlebot
        self.requestService(motionControl_move, (0.0, 0.0))

        # Tell mission control the target has been located.
        rospy.loginfo('locateTarget: target located.')
        self.requestService(mc_updateBelief, ("targetLocated", 1))

