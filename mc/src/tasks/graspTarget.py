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

class graspTarget(task):

    def __init__(self):
        pass

    def task(self, statusServices=[]):
        """Grasp the target."""
        pub = rospy.Publisher('belief_targetGrasped', belief_msg)
        pub.publish(belief_msg('targetGrasped', 1))
        rospy.loginfo("**** I grasped the target!")
