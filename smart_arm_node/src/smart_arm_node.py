from __future__ import division

# Basics
import ipdb
import roslib
roslib.load_manifest('smart_arm_node')
import rospy

# ROS messages
from std_msgs.msg import Float64, Boolean
from dynamixel_msgs.msg import JointState, MotorStateList

# Maths
from numpy import pi, arctan2, cos, sin, array, sqrt, arccos, eye, array, dot

################################################################################
def check_j1(msg):
    check_joint(1,msg)
def check_j2(msg):
    check_joint(2,msg)
def check_j3(msg):
    check_joint(3,msg)
def check_j4(msg):
    check_joint(4,msg)
def check_j5(msg):
    check_joint(5,msg)
    
    
    
################################################################################
def check_joint(n,msg):
    '''
    Checks if a joint is still moving and also if the torque is too big and the
    movement should be stopped.
    '''
    check_joint(1,msg)
################################################################################
def get_arm_limits():
    '''
    Obtains the limits from the parameter server of the "smart_arm_controller"
    '''
    rospy.get_param(param_name)
    
    
################################################################################
class Link(object):
    '''
    Class to handle revoution links defined with DH method.
    '''
    
    def __init__(self,theta,d,a,alpha,offset):
        '''
        Constructor of the class link.
        theta,d,a,alpha : defined by DH method.
        offset          : difference between the real zero of the joint motors
                          and the theoric zero.
        '''
        self.theta = theta
        self.d = d
        self.a = a
        self.alpha = alpha
        self.offset = offset
        
        
    def tr(self):
        '''
        Returns the forward kinematics matrix of the link at the design zero.
        '''
        self.tr(self.theta)
        
        
    def tr(self,q=0):
        '''
        Returns the forward kinematics matrix of the link at the especified
        theta (q) value.
        '''
        q = q-self.offset    # correct offset
        sa = sin(self.alpha)
        ca = cos(self.alpha)
        st = sin(q)
        ct = cos(q)
        d = self.d
        a = self.a
        val = array([[ct, -ca*st,  sa*st, a*ct],
                     [st,  ca*ct, -sa*ct, a*st],
                     [ 0,     sa,     ca,    d],
                     [ 0,      0,      0,    1]])
        return val



################################################################################ 
class Arm(object):
    '''
    Class to handle the whole arm of the robot with forward kinemtics and
    inverse kinematics.
    '''
    
    def __init__(self,links):
        '''
        Constructor of the arm with the specified list of links.
        '''
        assert isinstance(links,list)
        self.links = links
        self.n = len(links)
        self.limits = get_arm_limits()
    
    
    def fkine(self,qs):
        '''
        Returns the forward kinematics matrix of the whole arm by specifying
        a list of joint positions.
        '''
        assert len(qs)==self.n
        val = eye(4)
        for i in range(self.n):
            val = dot(val, self.links[i].tr(qs[i]))
        return val
        
        
    def ikine(self,x,y,z):
        '''
        Returns the inverse kinematics (list of joint positions) to achieve a
        (x,y,z) position on the end effector.
        '''
        ipdb.set_trace()
        limits = self.limits
        dummy = (z - d1)**2 + (sqrt(x**2 + y**2) - a1**2)
        q1 = arctan2(y,x)
        q2 = 1
        q3 = arccos( - (dummy - a2**2 - d4**2) / (2*a2*d4) ) # two solutions (q2, 2pi-q2)
        q4 = 0
        # q2 in limits
        if True:
            q2 = 1
        # q2 not in limits
        else:
            q3 = 2*pi - q3
            q2 = 1
        return [q1,q2,q3,q4]
    
    
    def move(self,qs):
        '''
        Moves the arm to the specified joint positions.
        '''
        assert isinstance(qs,list)
        assert len(qs)==self.n
        # Send movement commands to the smart_arm_controller
        for i in range(4):
            pub_move[i].publish(qs[i])
            
        # Wait until finish movement
        
        # Check for torques
        
        
    def move(self,x,y,z):
        '''
        Moves the arm to specified (x,y,z) position of the end effector, using
        the inverse kinematics solution.
        '''
        qs = self.ikine(x,y,z)
        self.move(qs)



################################################################################
# Publishers
pub_j1 = rospy.Publisher("/shoulder_pan_controller/command",   Float64)
pub_j2 = rospy.Publisher("/shoulder_pitch_controller/command", Float64)
pub_j3 = rospy.Publisher("/elbow_flex_controller/command",     Float64)
pub_j4 = rospy.Publisher("/wrist_roll_controller/command",     Float64)
pub_j5 = rospy.Publisher("/right_finger_controller/command",   Float64)
pub_move = [pub_j1, pub_j2, pub_j3, pub_j4, pub_j5]

move_state = rospy.Publisher("/smart_arm_node/move/state", Boolean)
grab_state = rospy.Publisher("/smart_arm_node/grab/state", Boolean)
    
# Subscribers
stat_j1 = rospy.Subscriber('/shoulder_pan_controller/state', JointState, check_j1)
stat_j2 = rospy.Subscriber('/shoulder_pan_controller/state', JointState, check_j2)
stat_j3 = rospy.Subscriber('/shoulder_pan_controller/state', JointState, check_j3)
stat_j4 = rospy.Subscriber('/shoulder_pan_controller/state', JointState, check_j4)
stat_j5 = rospy.Subscriber('/shoulder_pan_controller/state', JointState, check_j5)

move_com = rospy.Subscriber('/smart_arm_node/move/command', JointState, check_j1)
grab_com = rospy.Subscriber('/smart_arm_node/grab/command', JointState, check_j1)

# Sizes obtained with DH method
a1 = 0.06
a2 = 0.175
d1 = 0.14
d4 = 0.12

# Link creation
links = list()
links.append( Link(   0,  d1, a1, -pi/2,     0) )
links.append( Link(   0,   0, a2,    pi, -pi/4) ) # offset=-45
links.append( Link(pi/2,   0,  0, -pi/2,     0) )
links.append( Link(   0, -d4,  0,    pi,     0) )

# Robot creation
arm = Arm(links)

# Predefined positions
pos_offset = [0,-pi/4,0,0]
pos_horizontal = [0,0,pi/2,0]

# Forward kinematics
print arm.fkine(pos_horizontal)
print arm.ikine(0.1,0.1,0.1)
    

ipdb.set_trace()
