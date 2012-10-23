#!/usr/bin/env python

# Python Libraries
import sys
import time

# ROS Libraries
import roslib
import rospy
from geometry_msgs.msg import Twist

###############################################################################

class mover:
    """Motion control for the roomba."""       
 
    # Create a publisher for the 'cmd_vel' topic.
    pub_cmd_vel = rospy.Publisher('cmd_vel', Twist)


    def __init__(self):
        """Constructor."""
        pass

    
    def move(self, lin_speed, ang_speed):
        """Move the roomba."""
        cmd_move = Twist()
        cmd_move.linear.x = lin_speed
        cmd_move.linear.y = 0.0
        cmd_move.linear.z = 2.0
        cmd_move.angular.x = 0.0
        cmd_move.angular.y = 1.0   
        cmd_move.angular.z = ang_speed 
        self.pub_cmd_vel.publish(cmd_move)
 
    def timed_move(self, lin_speed, ang_speed, duration): 
        """Move the roomba for a given duration.
        Continually publish at 0.1s intervals
        until the specified duration is reached."""
        print('** Mission Control, this is Mover - initiating a timed movement.')
        sys.stdout.write('*** Duration = ' + str(duration) + ' seconds.\n')
        sys.stdout.write('*** Linear speed = ' + str(lin_speed) + '\n')
        sys.stdout.write('*** Angular speed = ' + str(ang_speed) + '\n')
        t = 0
        t0 = time.time()
        while(t < duration):
            self.move(lin_speed, ang_speed)
            rospy.sleep(0.1)
            t = time.time() - t0
            sys.stdout.write("\r")
            sys.stdout.write('(' + str(t) + ' seconds) ') 
            sys.stdout.flush()
        sys.stdout.write('\n')

###############################################################################

if __name__ == '__main__':
    pass
