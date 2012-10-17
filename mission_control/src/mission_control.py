#!/usr/bin/env python

# Python Libraries

# ROS Libraries
import roslib
roslib.load_manifest('mission_control')
import rospy

# Mission Control Libraries
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

def system_check():
    print("* System check:")
###############################################################################

if __name__ == '__main__':
    banner()
    rospy.init_node('mission_control')  # Initialize the node
   
    system_check() 
    
    exit_banner()
