#!/usr/bin/env python

# Python modules

# ROS modules
import roslib
#roslib.load_manifest('mc')
import rospy

# Custom modules
from task import task
from mc.msg import belief_msg

###############################################################################

class calibrateCamera(task):

    def __init__(self):
        pass

    def task(self, statusServices=[]):
        """Calibrate the camera."""
        pub = rospy.Publisher('belief_cameraCalibrated', belief_msg)
        pub.publish(belief_msg('cameraCalibrated', 1))
        rospy.loginfo("**** Camera calibrated.")
