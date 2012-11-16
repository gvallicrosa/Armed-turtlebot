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

class gotoTarget(task):

    name = "gotoTarget"

    def __init__(self):
        pass

    def task(self, statusServices=[]):
        """Approach the target."""
        self.requestService(updateBelief, ("atTarget", 1))
