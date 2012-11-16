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

class gotoTarget(task):

    def __init__(self):
        pass

    def task(self, statusServices=[]):
        """Approach the target."""
        pub = rospy.Publisher('belief_atTarget', belief_msg)
        pub.publish(belief_msg('atTarget', 1))
        rospy.loginfo("**** I'm at the target!")

