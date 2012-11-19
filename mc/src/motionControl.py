#!/usr/bin/env python

# Python Libraries
import time

# ROS Libraries
import roslib
roslib.load_manifest('motionControl')
import rospy
from geometry_msgs.msg import Twist

# Mission Control Libraries
from mc.srv import motionControl_move
from mc.srv import motionControl_timedMove

################################################################################

class motionControl:

    # Create a publisher for the 'cmd_vel' topic.
    pub_cmd_vel = rospy.Publisher('cmd_vel', Twist)

    def __init__(self):
        pass

    ############################################################################

    def launchServices(self):
        rospy.Service('motionControl_move', motionControl_move, self.move)
        rospy.Service('motionControl_timedMove', motionControl_timedMove, self.timedMove)
        rospy.spin()

    ############################################################################
   
    def movement(self, lin_speed, ang_speed):
        """Move the roomba."""
        rospy.loginfo('MotionControl: move.')
        rospy.loginfo('MotionControl: Linear speed = ' + str(lin_speed))
        rospy.loginfo('MotionControl: Angular speed = ' + str(ang_speed))
        cmd_move = Twist()
        cmd_move.linear.x = lin_speed
        cmd_move.linear.y = 0.0
        cmd_move.linear.z = 0.0
        cmd_move.angular.x = 0.0
        cmd_move.angular.y = 0.0   
        cmd_move.angular.z = ang_speed 
        self.pub_cmd_vel.publish(cmd_move)
   
   ############################################################################

    def move(self, msg):
        self.movement(msg.linear, msg.angular)
        return []

    ############################################################################
 
    def timedMove(self, msg):
        """Move the roomba for a given duration.
        Continually publish at 0.1s intervals
        until the specified duration is reached."""
        
        duration = msg.duration
        lin_speed = msg.linear
        ang_speed = msg.angular
       
        rospy.loginfo('MotionControl: timedMove.')
        rospy.loginfo('MotionControl: Duration = ' + str(duration) + ' seconds.')
        rospy.loginfo('MotionControl: Linear speed = ' + str(lin_speed))
        rospy.loginfo('MotionControl: Angular speed = ' + str(ang_speed))
        t = 0
        t0 = time.time()
        while(t < duration):
            self.movement(lin_speed, ang_speed)
            rospy.sleep(0.1)
            t = time.time() - t0
        return []

################################################################################

# Entry point >
if __name__ == "__main__":
    rospy.init_node('motionControl')
    mc = motionControl()
    mc.launchServices()
