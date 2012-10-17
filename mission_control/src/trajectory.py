#!/usr/bin/env python

# Python Libraries

# ROS Libraries
import roslib
roslib.load_manifest('mission_control')
import rospy

###############################################################################

class Trajectory:

    def __init__(self):
        self.trajectory = []

    def append(self, lin_speed, ang_speed, duration):
        step = []
        step.append(lin_speed)
        step.append(ang_speed)
        step.append(duration)
        self.trajectory.append(step)
