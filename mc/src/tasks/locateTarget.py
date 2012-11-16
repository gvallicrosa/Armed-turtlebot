#!/usr/bin/env python

# Python modules

# ROS modules
import roslib
#roslib.load_manifest('mc')
import rospy

# Custom modules
from task import task

###############################################################################

class locateTarget(task):

    def __init__(self):
        pass

    def task(self, statusServices=[]):
        """Locate the target."""
        rospy.loginfo("* Locating target...")
