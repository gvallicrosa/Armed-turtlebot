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

class locateTarget(task):

    name = "locateTarget"

    def __init__(self):
        pass

    def task(self, statusServices=[]):
        """Locate the target."""
        self.requestService(updateBelief, ("targetLocated", 1))
