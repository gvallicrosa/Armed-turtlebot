#!/usr/bin/env python

# Python Libraries

# ROS Libraries
import roslib
roslib.load_manifest('mission_control')
import rospy

# Mission Control Libraries
import font
from mission_control.srv import emergencyStop
from mission_control.srv import generatePlan
from mission_control.srv import executePlan
from smart_arm_node.srv import SmartArmService
from mission import mission

###############################################################################

class missionControl:

    def __init__(self):
        m = mission()

    def banner(self):
        """
        Displays a welcome banner in the terminal.
        """

        print('\n' + font.red + "===================================================================")
        print("              <<-- " + font.normal + font.underline + "M.I.S.S.I.O.N  C.O.N.T.R.O.L" +
            font.normal + font.red + " -->>")
        print("===================================================================" + font.normal)


    def exit_banner(self):
        """
        Displays an exit banner in the terminal.
        """

        print(font.red + "===================================================================" +
            font.normal + '\n')

    def activate(self):
        """
        This function is in oultimate control of the entire system.
        """

        # Display the welcome banner.
        self.banner()

        # Load the mission from a file.
        self.m.load("myMission.txt")

        # Get the number of tasks in the mission.
        # NOTE - this is not yet implemente din the 'mission' class.
        n = self.m.getNumTasks()

        # Loop until all tasks have been completed.
        while(True)
            self.m.RunNextTask()
            x = self.GetExitStatus()
            if( x == 1 ):
                print 'Something bad happened'
                # Do something about it...

        # Dispaly the exit banner.
        self.exit_banner()

###############################################################################

if __name__ == '__main__':

  # Create a missionControl and activate it.
  mc = missionControl()
  mc.activate()

  ##### TODO-
  ##### 0. import/implement 'mission' class
  ##### 1. create 'mission' member within this class
  ##### 2 .create small exemplar library of common taks by using clever combinations of
  ##### request for services and some intermediate control within the task.
  ##### 3. Factor the code below into some tasks.
  ##### 4. Sort out folder structure. Maybe 'tasks' package, and separate packages for each controller.
  ##### N.b. the whole project consists of a few controllers which are nodes that interface with the hardware.
  ##### N.b. The mission control should only care about mission tasks - completing
  ##### them and cycling to the next. It is not concerned with how those tasks are implemented.
  ##### N.b. The speicif details are encapsulated within the 'task' derived classes.

#  # Disaply the welcome banner.
#    banner()
#
#    # Register THIS node.
#    rospy.init_node('mission_control')
#
#    # *** BEGIN MISSION *** #
#
#    ## TODO - all mission 'TASKS' will be
#    ## wrapped in a function / 'MISSION' object.
#
#    # The first thing to do is cotact each node
#    # using this xxxStatus services, and
#    # make sure all nodes are OK.
#    ### TODO - implement a handler for BAD nodes.
#    systemCheck()
#
#    # There is no mission at the moment,
#    # so I will just demonstrate how to call
#    # servicees on the other nodes.
#    requestService(emergencyStop)  # Tell 'motionControl' to 'stop the robot'.
#    requestService(generatePlan)  # Tell 'motionControl' to 'generate a plan'.
#    requestService(executePlan)  # Tell 'motionControl' to 'execute a plan'.
#
#    # Example: set the joint angles of servos 1-4.
#    # param[1] = 2 --> 'type 2' request to 'get_arm_srv'.
#    # param[2] = [...] --> array of joint angles.
#    params = (2, [10.0, 20.0, 30.0, 40.0])  # Each tuple item is a service request parameter.
#   requestService(SmartArmService, params)  # Tell 'SmartArmServer' to 'set joint angles 1-4'.
#
#    # *** END MISSION *** #
#
#    # Display exit banner.
#    exit_banner()
