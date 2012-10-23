#!/usr/bin/env python

# Python Libraries

# ROS Libraries
import roslib
roslib.load_manifest('mission_control')
import rospy

# Mission Control Libraries
from mover import Mover
from trajectory import Trajectory

###############################################################################

class Motion_Controller:

    def __init__(self):
        pass

    def run(self, trajectory):
        print("** Mission Control, this is Motion Control - starting trajectory.")
        m = Mover()  # Create a 'mover' object.
        for step in trajectory:
            lin_speed = step[0]
            ang_speed = step[1]
            duration = step[2]
            m.timed_move(lin_speed, ang_speed, duration) # Move the roomba.    
    

###############################################################################

if __name__ == "__main__":

    rospy.init_node('motion_control')  # Initialize the node
    print("** Mission Control, this is Motion Control - node initialized.")

    tr = Trajectory()
    tr.append([1.0, 0.0, 0.0], [0.0, 0.0, 0.0], 5)
    tr.append([0.0, 1.0, 0.0], [0.0, 0.0, 0.0], 3)
    tr.append([0.0, 0.0, 0.0], [1.0, 0.0, 0.0], 2)

    mc = Motion_Controller()
    mc.run(tr.trajectory)
