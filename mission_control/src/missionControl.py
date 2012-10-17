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

def systemCheck():
    rospy.loginfo("Starting 'system check'...")

def requestService(service):
    serviceName = service.__name__    
    status = 0
    rospy.wait_for_service(serviceName)
    try:
        rq = rospy.ServiceProxy(serviceName, service)
        reply = rq()
        status = reply.status
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e
        status = 1
    if status == 0:
        rospy.loginfo("Request for '" + serviceName + "' was successful.")
    else:
        rospy.loginfo("Request for '" + serviceName + "' was unsuccessful.")


###############################################################################

if __name__ == '__main__':
    banner()
    rospy.init_node('mission_control')
   
    # BEGIN MISSION #

    systemCheck() 
    requestService(emergencyStop)
    requestService(generatePlan)
    requestService(executePlan)

    # END MISSION #

    exit_banner()
