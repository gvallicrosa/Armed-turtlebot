#!/usr/bin/python
# -*- coding: utf-8 -*-
from __future__ import division

# ROS basics
import roslib
roslib.load_manifest('sinus_arm')
import rospy

# ROS messages
from std_msgs.msg import Float64

# Maths and sleep
from numpy import sin, pi, arange
from time import sleep



################################################################################
def main():
    # Initialize ROS node
    rospy.init_node('UWS_driver', anonymous=True)
    rospy.loginfo('UWS_driver node initialized')
    
    # Arm publishers
    pub_grasp = rospy.Publisher("/right_finger_controller/command", Float64)
    pub_elbow_flex = rospy.Publisher("/elbow_flex_controller/command", Float64)
    pub_wrist_roll = rospy.Publisher("/wrist_roll_controller/command", Float64)
    pub_shoulder_pan = rospy.Publisher("/shoulder_pan_controller/command", Float64)
    pub_shoulder_pitch = rospy.Publisher("/shoulder_pitch_controller/command", Float64)
    
    # Sinus movement
    for t in arange(1e3):
        val = sin(2*pi*t/100)
        pub_wrist_roll.publish(Float64(val))
        pub_grasp.publish(Float64(val))
        sleep(0.05)
    
    # Arm subscribers
#    /diagnostics
#    /elbow_flex_controller/state
#    /motor_states/smart_arm
#    /right_finger_controller/state
#    /shoulder_pan_controller/state
#    /shoulder_pitch_controller/state
#    /wrist_roll_controller/state

############################################################################################################
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
