#!/usr/bin/env python

# Python modules

# ROS modules
import roslib
roslib.load_manifest('mc')
import rospy

# Custom modules

###############################################################################

class task:

    def __init__(self):
        pass

    name = "(user was too lazy to give me a name)"

    def requestService(self, service, params=None):
        """
        Thin wrapper for generic service requests.

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

        # Appropriate services will have to be imported on a per task basis.
        serviceName = service.__name__

        # See if the service is available. If not, wait until it is.
        rospy.wait_for_service(serviceName)

        # Attempt to use the service.
        try:
            rq = rospy.ServiceProxy(serviceName, service)
            rq(*params)  # Expand the 'params' tuple into an argument list.
        except rospy.ServiceException:
            rospy.logwarn("Service call failed")

    def start(self):
        self.task()

    def task(self):
        """
        This will be implemented in the derived classes.
        It should consist of calls to __requestService().
        """
        pass
