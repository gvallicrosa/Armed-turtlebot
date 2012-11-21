#!/usr/bin/env python

# Python modules
import os

# ROS modules
import roslib
#roslib.load_manifest('mc')
import rospy

# Custom modules
from task import task
from mc.srv import mc_updateBelief
from mc.srv import vs_graspTarget
from std_msgs.msg import String

###############################################################################

class graspTarget(task):

    name = "graspTarget"

    def __init__(self):
        # Assume that the target is not grasped.
        self.grasped = 0
        FivePointsString = ''

    ###########################################################################

    def update5PointsHandler(self, msg):
        self.FivePointsString = msg

    ###########################################################################

    def task(self, statusServices=[]):
        """Grasp the target."""

        # Listen to the detection node - get the radius of the ball and 5-points string.
        rospy.Subscriber('/detection/5Points', String, self.update5PointsHandler)

        # Start the visual servoing.
        os.system('roslaunch tracking tracker.launch')
        
        # Post string to ROS param server
        rospy.set_param('circleInit', FivePointsString)
        
        # Tell mission control the target has been grasped.
        self.requestService(mc_updateBelief, ("targetGrasped", int(self.grasped)))
