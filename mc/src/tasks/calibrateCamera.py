#!/usr/bin/env python

# Python modules
import os

# ROS modules
import roslib
#roslib.load_manifest('mc')
import rospy

# Custom modules
from task import task
from mc.srv import mc_updateBelief
###############################################################################

class calibrateCamera(task):

    name = "calibrateCamera"

    def __init__(self):
        pass

    def task(self, statusServices=[]):
        """Calibrate the camera."""

        #######################################################################
        #                 DO NOT EDIT CODE ABOVE THIS LINE                    #
        #######################################################################

        # CHANGE THE FOLLOWING TWO VARIABLES AS NECESSARY
        command = "./calibrateInt -p"
        image = "/home/user/calibData/img-%02d.pgm"

        #######################################################################
        #                 DO NOT EDIT OCDE BELOW THIS LINE                    #
        #######################################################################

        result = os.system(command + image)
        rospy.loginfo("calibrateCamera: Camera calibrates.")
        self.requestService(mc_updateBelief, ("cameraCalibrated", 1))
