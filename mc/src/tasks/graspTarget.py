#!/usr/bin/env python

# Python modules

# ROS modules
import roslib
#roslib.load_manifest('mc')
import rospy

# Custom modules
from task import task

###############################################################################

class graspTarget(task):

    def __init__(self):
        pass

    def task(self, statusServices=[]):
        """Grasp the target."""
        rospy.loginfo("* Grasping target...")
