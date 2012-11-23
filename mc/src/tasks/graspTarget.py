#!/usr/bin/env python

# Python modules
import os
import subprocess

# ROS modules
import rospy

# Custom modules
from task import task
from mc.srv import mc_updateBelief
from std_msgs.msg import String

###############################################################################

class graspTarget(task):

    name = "graspTarget"

    def __init__(self):
        # Assume that the target is not grasped.
        self.grasped = 0
        self.fivePointsString = '0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0'

    ###########################################################################

    def update5PointsHandler(self, msg):
        self.fivePointsString = msg

    ###########################################################################

    def task(self, statusServices=[]):
        """Grasp the target."""

        # Listen to the detection node - get the radius of the ball and 5-points string.
        rospy.Subscriber('/detection/5Points', String, self.update5PointsHandler)

        # We can either i) wait for a message telling us that the target has
        # been grasped or ii) assume that the attempt will be successful.
        self.grasped = 1 # ii)

        # Start the visual servoing.
        os.system("xterm -geometry 100x50 -hold -T 'roslaunch tracking tracker.launch' -e 'roslaunch tracking tracker.launch'")

        # Post string to ROS param server
        rospy.set_param('circleInit', self.fivePointsString)

        # Tell mission control the target has been grasped.
        self.requestService(mc_updateBelief, ("targetGrasped", int(self.grasped)))
