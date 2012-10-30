#!/usr/bin/env python

# Python Libraries

# ROS Libraries
import roslib
roslib.load_manifest('mission_control')
import rospy

# Mission Control Librariesi
import task
# from mission_control.srv import emergencyStop
# from mission_control.srv import generatePlan
# from mission_control.srv import executePlan
# from smart_arm_node.srv import SmartArmService

###############################################################################


class mission:

    def __init__(self):
        currentTask = 0  # Keep track of how many tasks have completed.
        totalTasks = 0  # Keep track of how many tasks there are in the mission.
        exitStatus = 0  # Keep track of how the last task exited.

        # Think of some way to populate the task dictionary using
        # a scan of the 'task' package.
        # I.e. {class_name; class}
        task_dictionary = {}

        ### Setup a parser to read the text file for the load() function.

    def getExitStatus(self):
        """
        Return the exit status of the LAST task that executed.
        This is for the benefit of mission_control.
        It may need to change the mission.
        """
        return self.exitStatus

    def load(self, filename='mission.txt'):
        """
        Load data from a text file
        and generate the mission using the python parser class
        and a dictionary of 'taskName: class' pairs.
        """
        pass

    def reset(self):
       """
       === Not essential ===
       Removes all entries from the mission
       """
       pass

   def remove(self, taskNumber):
       """
       === Not essential ===
       Remove specific task from mission
       """
       pass

   def insert(self, tasknNumber, taskName);
       """
       === Not essential ===
       Insert taskName after taskNumber
       """
       pass

   def list(self):
       """
       Print an ordered list of the mission tasks
       """
       pass

   def StartNextTask(self):
       """
       Executes the next task.
       Increments the task counter on complete
       """
       pass
