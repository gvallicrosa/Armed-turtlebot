#!/usr/bin/env python

# Python Libraries

# ROS Libraries
import roslib
roslib.load_manifest('mission_control')
import rospy

# Mission Control Libraries
import font
from mission_control.srv import *

###############################################################################

def banner():
    print('\n' + font.red + "===================================================================")
    print("              <<-- " + font.normal + font.underline + "M.I.S.S.I.O.N  C.O.N.T.R.O.L" +
           font.normal + font.red + " -->>")
    print("===================================================================" + font.normal)

def exit_banner():
    print(font.red + "===================================================================" +
          font.normal +'\n')

def t_systemCheck():
    rospy.loginfo("Starting 'system check'...")

def t_emergencyStop():
    status = 0
    rospy.wait_for_service('emergencyStop')
    try:
        rq_emergencyStop = rospy.ServiceProxy('emergencyStop', emergencyStop)
        reply = rq_emergencyStop()
        status = reply.status
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e
        status = 1
    if status == 0:
        rospy.loginfo("Request for 'emergency stop' was successful.")
    else:
        rospy.loginfo("Request for 'emergency stop' was unsuccessful.")

def t_generatePlan():
    status = 0
    rospy.wait_for_service('generatePlan')
    try:
        rq_generatePlan = rospy.ServiceProxy('generatePlan', generatePlan)
        reply = rq_generatePlan()
        status = reply.status
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e
        status = 1
    if status == 0:
        rospy.loginfo("Request for 'generate plan' was successful.")
    else:
        rospy.loginfo("Request for 'generate plan' was unsuccessful.")

def t_executePlan():
    status = 0
    rospy.wait_for_service('executePlan')
    try:
        rq_executePlan = rospy.ServiceProxy('executePlan', executePlan)
        reply = rq_executePlan()
        status = reply.status
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e
        status = 1
    if status == 0:
        rospy.loginfo("Request for 'execute plan' was successful.")
    else:
        rospy.loginfo("Request for 'execute plan' was unsuccessful.")

###############################################################################

if __name__ == '__main__':
    banner()
    rospy.init_node('mission_control')
   
    # BEGIN MISSION #

    t_systemCheck() 
    t_emergencyStop()
    t_generatePlan()
    t_executePlan()

    # END MISSION #

    exit_banner()
