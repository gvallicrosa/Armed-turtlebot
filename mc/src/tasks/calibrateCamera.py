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

class calibrateCamera(task):

    name = "calibrateCamera"

    def __init__(self):
        pass

    def task(self, statusServices=[]):
        """Calibrate the camera."""
        self.requestService(updateBelief, ("cameraCalibrated", 1))
