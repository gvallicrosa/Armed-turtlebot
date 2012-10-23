#!/usr/bin/env python

# Python Libraries

# ROS Libraries
import roslib
roslib.load_manifest('mission_control')
import rospy

# Mission Control Libraries
from mover import mover

###############################################################################

class motionPlan:

    def __init__(self):
        self.mover = mover()
        self.plan = []
    
    def generate(self, trajectory):
        pass

    def update(self):
        pass

    def display(self):
        pass

    def execute(self):
        # Iterate through each step in the motion plan.
        for step in self.plan:
            # Move the roomba.
            m.move(step[0], step[1])   
