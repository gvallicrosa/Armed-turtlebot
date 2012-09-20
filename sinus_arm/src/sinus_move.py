#!/usr/bin/python
# -*- coding: utf-8 -*-
from __future__ import division

# ROS basics
import roslib
roslib.load_manifest('sinus_arm')
import rospy

# ROS messages
from std_msgs.msg import Float64
from dynamixel_msgs.msg import JointState, MotorStateList

# Maths and sleep
from numpy import sin, pi, arange
from time import sleep
import random



################################################################################
#def sub_grasp(msg):
#def sub_elbow_flex(msg):
#def sub_wrist_roll(msg):
#def sub_shoulder_pan(msg):
#def sub_shoulder_pitch(msg):



################################################################################
def main():
    # Initialize ROS node
    rospy.init_node('UWS_driver', anonymous=True)
    rospy.loginfo('UWS_driver node initialized')
    
    # Arm publishers (control the arm)
    pub_grasp = rospy.Publisher("/right_finger_controller/command", Float64)
    pub_elbow_flex = rospy.Publisher("/elbow_flex_controller/command", Float64)
    pub_wrist_roll = rospy.Publisher("/wrist_roll_controller/command", Float64)
    pub_shoulder_pan = rospy.Publisher("/shoulder_pan_controller/command", Float64)
    pub_shoulder_pitch = rospy.Publisher("/shoulder_pitch_controller/command", Float64)
    
    # Sinus movement
    for t in arange(1e3):
#        val = 0.5*sin(2*pi*t/100)
        val = (random.random()*1.7 -1)
        pub_wrist_roll.publish(Float64(val))
        pub_grasp.publish(Float64(val))
        pub_elbow_flex.publish(Float64(val))
        pub_wrist_roll.publish(Float64(val))
        pub_shoulder_pan.publish(Float64(val))
        pub_shoulder_pitch.publish(Float64(val))
        sleep(2) # enough time to move to the random position
    
    # Arm subscribers (to check joint load, velocities...)
#    rospy.Subscriber('/right_finger_controller/state',   JointState, sub_grasp)
#    rospy.Subscriber('/elbow_flex_controller/state',     JointState, sub_elbow_flex)
#    rospy.Subscriber('/wrist_roll_controller/state',     JointState, sub_wrist_roll)
#    rospy.Subscriber('/shoulder_pan_controller/state',   JointState, sub_shoulder_pan)
#    rospy.Subscriber('/shoulder_pitch_controller/state', JointState, sub_shoulder_pitch)




############################################################################################################
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
