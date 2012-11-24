#!/usr/bin/env python

# Python modules
import os
import subprocess

# ROS modules
import rospy

# Custom modules
from task import task
from mc.srv import mc_updateBelief
from std_msgs.msg import String

# Smart arm server
from smart_arm_node.srv import SmartArmService

###############################################################################

class graspTarget(task):

    name = "graspTarget"

    def __init__(self):
        # Assume that the target is not grasped.
        self.grasped = 0
        self.fivePointsString = '0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0'

    ###########################################################################

    def update5PointsHandler(self, msg):
        self.fivePointsString = msg

    ###########################################################################

    def task(self, statusServices=[]):
        """Grasp the target."""

        # Listen to the detection node - get the radius of the ball and 5-points string.
        rospy.Subscriber('/detection/5Points', String, self.update5PointsHandler)

        # We can either i) wait for a message telling us that the target has
        # been grasped or ii) assume that the attempt will be successful.
        self.grasped = 1 # ii)

        # Start the visual servoing.
        rospy.set_param('/visualservoing/circleInit', self.fivePointsString)
        os.system("xterm -geometry 100x50 -hold -T 'roslaunch tracking tracker.launch' -e 'roslaunch tracking tracker.launch'")
        
        # Check the ball position respect to the arm base
        trans, rot = tf.TransformListener().lookupTransform("/joint0", "/obj_pos", now)
        rospy.wait_for_service('/arm_server/services')
        try:
            use_service = rospy.ServiceProxy('/arm_server/services', SmartArmService)
            resp = use_service(7, trans)
            ans = resp.values[0]
        except rospy.ServiceException, e:
            rospy.logerr(("Service call failed: %s" % e))
        
        # If the position is reachable the arm will move
        if ans:
            # Grasp the ball
            rospy.wait_for_service('/arm_server/services')
            try:
                use_service = rospy.ServiceProxy('/arm_server/services', SmartArmService)
                resp = use_service(8, [0,])
                ans = resp.values[0]
            except rospy.ServiceException, e:
                rospy.logerr(("Service call failed: %s" % e))
            # Back to home position
            rospy.wait_for_service('/arm_server/services')
            try:
                use_service = rospy.ServiceProxy('/arm_server/services', SmartArmService)
                resp = use_service(10, [0,])
                ans = resp.values[0]
            except rospy.ServiceException, e:
                rospy.logerr(("Service call failed: %s" % e))
        else:
            self.grasped = 0        

        # Tell mission control the target has been grasped.
        self.requestService(mc_updateBelief, ("targetGrasped", int(self.grasped)))
