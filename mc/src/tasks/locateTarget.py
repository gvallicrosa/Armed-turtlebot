#!/usr/bin/env python

# Python modules

# ROS modules
import roslib
#roslib.load_manifest('mc')
import rospy
from std_msgs.msg import Int8
from std_msgs.msg import Float64

# Custom modules
from task import task
from mc.srv import mc_updateBelief
from mc.srv import motionControl_move
###############################################################################

class locateTarget(task):

    name = "locateTarget"

    def __init__(self):
        # Assume that we can't see the target.
        self.located = 0
        self.measuredRadius = 0

    ###########################################################################
        
    def updateRadiusHandler(self, msg):
        """Callback function for subscribes."""
        self.measuredRadius = msg 

    ###########################################################################

    def task(self, statusServices=[]):
        """Locate the target."""

        # Loop until the target is located.
        while(self.located == 0):

            # Listen out for the radius of the ball.
            rospy.Subscriber('/detection/radius', Float64, self.updateRadiusHandler)

            # Is the ball in the camera's field of view?
            if(self.measuredRadius != 0):
               self.located = 1 
               
               # Listen for the centroid co-ordinates and rotate to centre the ball
               # in the field of view of the camera.
            
            # Have we found the ball yet?
            if(self.located == 0):
                # Rotate the turtlebot
                self.requestService(motionControl_move, (0.0, 0.1))
                rospy.loginfo('locateTarget: Locating target...')

        # Stop the turtlebot
        self.requestService(motionControl_move, (0.0, 0.0))

        # Tell mission control the target has been located.
        rospy.loginfo('locateTarget: target located.')
        self.requestService(mc_updateBelief, ("targetLocated", 1))

