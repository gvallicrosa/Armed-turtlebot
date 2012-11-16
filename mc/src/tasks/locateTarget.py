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

class locateTarget(task):

    def __init__(self):
        pass

    def task(self, statusServices=[]):
        """Locate the target."""
        pub = rospy.Publisher('belief_targetLocated', belief_msg)
        pub.publish(belief_msg('targetLocated', 1))
        rospy.loginfo("**** Target located!")
