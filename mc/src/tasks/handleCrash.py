#!/usr/bin/env python

# Python modules

# ROS modules
import rospy

# Custom modules
from task import task
from mc.srv import motionControl_move
from mc.srv import motionControl_timedMove

###############################################################################

class handleCrash(task):

    def __init__(self):
        pass

    def task(self, statusServices=[]):
        """Handle crash scenario."""
        rospy.logdebug("handleCrash: Handling crash...")

        # 1. Reverse away from obstacle.
        self.requestService(motionControl_timedMove, (-0.1, 0.0, 3))

        # Stop
        self.requestService(motionControl_move, (0.0, 0.0))

        # 2. Turn away from obstacle.
        self.requestService(motionControl_timedMove, (0.0, 0.1, 3))

        # Stop
        self.requestService(motionControl_move, (0.0, 0.0))

        # 3. Move forwards.
        self.requestService(motionControl_timedMove, (0.1, 0.0, 3))

        # Stop
        self.requestService(motionControl_move, (0.0, 0.0))

        # NOTE: The turtlebot will need to relocate the target.
        self.requestService(mc_updateBelief, ("crashed", 0))
