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

class centreTarget(task):

    def __init__(self):
        pass

    def task(self, statusServices=[]):
        """Centre the target in the camera."""
        pub = rospy.Publisher('belief_targetCentred', belief_msg)
        pub.publish(belief_msg('targetCentred', 1))
        rospy.loginfo("**** Target centred!")
