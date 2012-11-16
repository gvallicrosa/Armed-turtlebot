#!/usr/bin/env python

# Python modules

# ROS modules
import roslib
#roslib.load_manifest('mc')
import rospy

# Custom modules
from task import task

###############################################################################

class centreTarget(task):

    def __init__(self):
        pass

    def task(self, statusServices=[]):
        """Centre the target in the camera."""
        rospy.loginfo("* Centering target...")
