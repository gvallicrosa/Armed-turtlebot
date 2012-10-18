#!/usr/bin/env python

# Python Libraries

# ROS Libraries
import roslib
roslib.load_manifest('mission_control')
import rospy

# Mission Control Libraries
from motionPlan import motionPlan
from mover import mover
from mission_control.srv import *

###############################################################################

class motionControl:

    def __init__(self):
        self.status = 0
        self.mp = motionPlan()

    def h_motionControlStatus(self, data):
        return motionControlStatusResponse(self.status)        

    def h_emergencyStop(self, data):
        status = 0
        try:
            m = mover()
            m.move(0.0, 0.0)
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
            status = 1
        return emergencyStopResponse(status)        

    def h_generatePlan(self, data):
        status = 0
        try:
            print('Pretending to generate a plan.')
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
            status = 1
        return generatePlanResponse(status)        

    def h_executePlan(self, data):
        status = 0
        try:
            print('Pretending to execute a plan.')
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
            status = 1
        return executePlanResponse(status)        

    def launchServices(self):
        s_motionControl = rospy.Service('motionControlStatus', motionControlStatus, self.h_motionControlStatus)
        s_emergencyStop = rospy.Service('emergencyStop', emergencyStop, self.h_emergencyStop)
        s_generatePlan = rospy.Service('generatePlan', generatePlan, self.h_generatePlan)
        s_executePlan = rospy.Service('executePlan', executePlan, self.h_executePlan)
        rospy.spin()

###############################################################################

# Entry point >
if __name__ == "__main__":
    rospy.init_node('motionControl')
    mc = motionControl()
    mc.launchServices()
