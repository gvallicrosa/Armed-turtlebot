#!/usr/bin/env python

# Python modules

# ROS modules
import roslib
#roslib.load_manifest('mc')
import rospy

# Custom modules
from task import task
from mc.srv import mc_updateBelief
from mc.srv import vs_graspTarget
###############################################################################

class graspTarget(task):

    name = "graspTarget"

    def __init__(self):
        # Assume that we can't see the target.
        self.grasped = 0

    ###########################################################################

    def targetGrasped(self, msg):
        self.grasped = msg

    ###########################################################################

    def task(self, statusServices=[]):
        """Grasp the target."""

        # Listen if the target has been located. (y/n)
        # This topic must be published by the camera node.
        rospy.Subscriber('vs_targetGrasped', Int8, self.targetGrasped)

        # Start the visual servoing.
        self.requestService(vs_graspTarget)

        # Tell mission control the target has been grasped.
        self.requestService(mc_updateBelief, ("targetGrasped", grasped))
