#!/usr/bin/env python

# Python Libraries
import time

# ROS Libraries
import roslib
import rospy
from geometry_msgs.msg import Twist
from mc.msg import TurtlebotSensorState

# Mission Control Libraries
from mc.srv import motionControl_move
from mc.srv import motionControl_timedMove

################################################################################

class motionControl:

    # Create a publisher for the 'cmd_vel' topic.
    pub_cmd_vel = rospy.Publisher('cmd_vel', Twist)

    def __init__(self):
        # Assume the turtlebot isn't in a crashed state.
        self.crash = 0

    ############################################################################

    def requestService(self, service, params=None):
        serviceName = service.__name__
        rospy.wait_for_service(serviceName)
        try:
            rq = rospy.ServiceProxy(serviceName, service)
            rq(*params)  # Expand the 'params' tuple into an argument list.
        except rospy.ServiceException:
            rospy.logwarn("Service call failed")

    ############################################################################

    def processSensing(self, TurtlebotSensorState):
        """Callback function responding to turtlebot sensor data."""

        # Has the turtlebot crashed?
        crashed = TurtlebotSensorState.bumps_wheeldrops
        if(crashed == 1):
            # Tell mission control.
            self.requestService('mc_updateBelief', ('crashed', 1))

            # Prevent move() and timedMove() from moving the base.
            self.crashed = 1

        else:     
            # Tell mission control.
            self.requestService('mc_updateBelief', ('crashed', 0))

            # Enable move() and timedMove().
            self.crashed = 0

    ############################################################################

    def launchServices(self):
        """Start services and subscribers"""
        rospy.Service('motionControl_move', motionControl_move, self.move)
        rospy.Service('motionControl_timedMove', motionControl_timedMove, self.timedMove)
        rospy.Subscriber('/turtlebot_node/sensor_state', TurtlebotSensorState, self.processSensing)
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
        
        # If the turtlebot has crashed then disable forwards motion.
        if(self.crashed == 1 and lin_speed > 0 and ang_speed == 0):
            # TRUE: Display warning.
            rospy.logwarn("motionControl: SAFEGUARD - crash detected - forwards motion disallowed.")
        else:
            # FALSE: Move.
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
