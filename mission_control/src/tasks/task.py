#!/usr/bin/env python

# Python Libraries

# ROS Libraries
import roslib
roslib.load_manifest('mission_control')
import rospy

# Mission Control Libraries
# from mission_control.srv import emergencyStop
# from mission_control.srv import generatePlan
# from mission_control.srv import executePlan
# from smart_arm_node.srv import SmartArmService

###############################################################################


class task:

    def __init__(self):
        pass

    def __requestService(self, service, params=None):
        """
        Thin wrapper for service requests.

        Example usage:

        >>> params = (2, [10, 20, 30], 'hello')
        >>> requestService(service_name, params)

        requestService will then expand 'params' and pass each
        tuple element to the serviceProxy object as follows:

        >>> rq = rospy.ServiceProxy(service_name, service)
        >>> reply = rq(2, [10, 20, 30], 'hello')

        'params' can also be omitted if the service request
        takes no parameters e.g.

        >>> requestService(service_name)
        """

        serviceName = service.__name__
        status = 0

        # See if the service is available. If not, wait until it is.
        rospy.wait_for_service(serviceName)

        # Attempt to use the service.
        try:
            rq = rospy.ServiceProxy(serviceName, service)
            reply = rq(*params)  # Expand the 'params' tuple into an argument list.
            status = reply.status
        except rospy.ServiceException, e:
            print "Service call failed: %s" % e
            status = 1

        # Tell the user if the service request was successful/unsuccessful.
        if status == 0:
            rospy.loginfo("Request for '" + serviceName + "' was successful.")
        else:
            rospy.loginfo("Request for '" + serviceName + "' was unsuccessful.")
        return reply

    def task(self):
        """
        This will be implemented in the derived classes.
        It should consist of calls to __requestService().
        """
        pass
