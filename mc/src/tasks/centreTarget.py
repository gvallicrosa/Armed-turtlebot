#!/usr/bin/env python

# Python modules

# ROS modules
import rospy

# Custom modules
from task import task
from mc.srv import updateBelief
###############################################################################

class centreTarget(task):

    name = "centreTarget"

    def __init__(self):
        pass

    def task(self, statusServices=[]):
        """Centre the target in the camera."""
        self.requestService(updateBelief, ("targetCentred", 1))
