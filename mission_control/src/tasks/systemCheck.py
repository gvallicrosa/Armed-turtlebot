#!/usr/bin/env python

# Python Libraries

# ROS Libraries
import roslib
roslib.load_manifest('mission_control')
import rospy

# Mission Control Libraries
from task import task
### E.g. from smart_arm_node.srv import SmartArmStatus

###############################################################################


class systemCheck(task):

    def __init__(self):
        pass

    def task(self, statusServices=[]):
        """
        Make sure all nodes are up and running.
        'statusServices' is an array of service names to call.

        Example usage:
        To check the status of the 'motionControl' and 'ArmControl' nodes:

        >>> statusServices = [motionControlStatus, ArmControlStatus]
        >>> systemCheck(statusServices)

        We assume that the 'motionControlStatus' and 'ArmControlStatus' services
        have been implemented as part of the 'motionControl' and 'ArmControl' nodes.
        """

        rospy.loginfo("Starting 'system check'...")
        for service in statusServices:
            self.requestService(service)
