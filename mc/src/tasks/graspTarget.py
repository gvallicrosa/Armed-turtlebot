#!/usr/bin/env python

# Python modules

# ROS modules
import roslib
#roslib.load_manifest('mc')
import rospy

# Custom modules
from task import task
from mc.srv import updateBelief
###############################################################################

class graspTarget(task):

    name = "graspTarget"

    def __init__(self):
        pass

    def task(self, statusServices=[]):
        """Grasp the target."""
        self.requestService(updateBelief, ("targetGrasped", 1))
