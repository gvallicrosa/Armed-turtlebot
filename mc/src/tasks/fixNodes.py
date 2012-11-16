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

class fixNodes(task):

    def __init__(self):
        pass

    def task(self, statusServices=[]):
        """Make sure all nodes are up and running."""
        pub = rospy.Publisher('belief_nodesOnline', belief_msg)
        pub.publish(belief_msg('nodesOnline', 1))
        rospy.loginfo("**** Nodes fixed.")
