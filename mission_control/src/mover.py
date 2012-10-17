#!/usr/bin/env python

# Python Libraries
import sys
import time

# ROS Libraries
import roslib
import rospy
from geometry_msgs.msg import Twist

###############################################################################

class Mover:
    """Motion control for the roomba."""       
 
    # Create a publisher for the 'cmd_vel' topic.
    pub_cmd_vel = rospy.Publisher('cmd_vel', Twist)


    def __init__(self):
        """Constructor."""
        pass

    
    def move(self, lin_speed, ang_speed):
        """Move the roomba."""
        cmd_move = Twist()
        cmd_move.linear.x = lin_speed[0]
        cmd_move.linear.y = lin_speed[1] 
        cmd_move.linear.z = lin_speed[2]
        cmd_move.angular.x = ang_speed[0]
        cmd_move.angular.y = ang_speed[1]   
        cmd_move.angular.z = ang_speed[2] 
        self.pub_cmd_vel.publish(cmd_move)

    
    def stop(self):
        """Stop the roomba."""
        print("** Mission Control, this is Mover - all stop.")
        cmd_stop = Twist() 
        self.pub_cmd_vel.publish(cmd_stop)
   
 
    def timed_move(self, lin_speed, ang_speed, duration): 
        """Move the roomba for a given duration.
        Continually publish at 0.1s intervals
        until the specified duration is reached."""
        print('** Mission Control, this is Mover - initiating a timed movement.')
        sys.stdout.write('*** Duration = ' + str(duration) + ' seconds.\n')
        sys.stdout.write('*** Linear speed = ('
                         + str(lin_speed[0]) + ', '
                         + str(lin_speed[1]) + ', ' 
                         + str(lin_speed[2]) + ')\n')
        sys.stdout.write('*** Angular speed = ('
                         + str(ang_speed[0]) + ', '
                         + str(ang_speed[1]) + ', ' 
                         + str(ang_speed[2]) + ')\n')
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
        self.stop()  # Stop the roomba.

###############################################################################

if __name__ == '__main__':
    lin_speed = [0.1, 0.0, 0.0]  # 0.1 m/s
    ang_speed = [0.0, 0.0, 0.0]  # 0.0 rad/s
    duration = 5.0  # 1.0s
    m = Mover()  # Create a 'mover' object.
    m.timed_move(lin_speed, ang_speed, duration) # Move the roomba.    
