#!/usr/bin/env python

# Python Libraries

# ROS Libraries
import roslib
roslib.load_manifest('mission_control')
import rospy

# Mission Control Libraries
from mover import Mover
import font

###############################################################################

def banner():
    print('\n' + font.red + "===================================================================")
    print("              <<-- " + font.normal + font.underline + "M.I.S.S.I.O.N  C.O.N.T.R.O.L" +
           font.normal + font.red + " -->>")
    print("===================================================================" + font.normal)

def exit_banner():
    print(font.red + "===================================================================" +
          font.normal +'\n')

###############################################################################

if __name__ == '__main__':
    banner()
    ########

    rospy.init_node('mission_control')  # Initialize the node
    print("* Houston, this is mission control - node initialized.")

    lin_speed = [0.1, 0.0, 0.0]  # 0.1 m/s
    ang_speed = [0.0, 0.0, 0.0]  # 0.0 rad/s
    duration = 5.0  # 1.0s

    m = Mover()  # Create a 'mover' object.
    m.timed_move(lin_speed, ang_speed, duration) # Move the roomba.    

    #############
    exit_banner()
