#!/usr/bin/env python

# Python modules

# ROS modules
import roslib
#roslib.load_manifest('mc')
import rospy

# Custom modules
from task import task

###############################################################################

class gotoTarget(task):

    def __init__(self):
        pass

    def task(self, statusServices=[]):
        """Approach the target."""
        rospy.loginfo("* Approaching target...")
