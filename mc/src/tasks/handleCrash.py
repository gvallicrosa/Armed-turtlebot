#!/usr/bin/env python

# Python modules

# ROS modules
import roslib
#roslib.load_manifest('mc')
import rospy

# Custom modules
from task import task

###############################################################################

class handleCrash(task):

    def __init__(self):
        pass

    def task(self, statusServices=[]):
        """Handle crash scenario."""
        rospy.loginfo("* Handling crash...")
