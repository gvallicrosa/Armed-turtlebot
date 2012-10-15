#from __future__ import division

# Basics
import ipdb
import roslib
roslib.load_manifest('smart_arm_node')
import rospy

# ROS messages
from std_msgs.msg import Float64, Bool, Float64MultiArray
from dynamixel_msgs.msg import JointState, MotorStateList
from smart_arm_node.msg import Float64List
from smart_arm_node.srv import SmartArmService, SmartArmServiceResponse

# Maths
from numpy import pi, arctan2, cos, sin, array, sqrt, arccos, eye, array, dot, zeros

# Time
from time import sleep



################################################################################
def ask_server(req):
    """
    Accepts external requests about status and movements, depending on the
    'req.what' value.
        1 : joint status
        2 : move the four first joints to desired joint angles
        3 : move the four first joints to desired (x,y,z) using IK solver
        4 : grab/ungrab with the hand
    Data should be provided for some of the requests.
    A response according to what is accomplished is retured.
    """
    if req.what == 1:
        # joint status
        ans = list(joints_curr)
    elif req.what == 2:
        # move radians
        qs = list(req.data)
        ans = [arm.move_all(qs),]
    elif req.what == 3:
        # move xyz
        xyz = list(req.data)
        ans = [arm.move_xyz(xyz),]
    elif req.what == 4:
        # grab/ungrab
        ans = [arm.grab(),]
    else:
        # no valid request
        print 'no valid request\n'
        ans = [0,]

    return SmartArmServiceResponse(ans)
        


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
def check_joint(i,msg):
    '''
    Checks if a joint is still moving and also if the torque is too big and the
    movement should be stopped.
    '''

    joints_move[i-1] = msg.is_moving
    joints_load[i-1] = abs(msg.load) > 0.8
    joints_curr[i-1] = msg.current_pos
    
    if joints_load[i-1] and i==5:
        print 'Too much load on joint %s\nStopping joint...' % i
        print 'Load = ', msg.load
        pub_move[i-1].publish(msg.current_pos)
    
    
    
################################################################################
def get_arm_limits():
    '''
    Obtains the limits for each joint from the parameter server
    of the "smart_arm_controller"
    '''
    # Names of the parameters to get
    tic2rad = rospy.get_param('/dynamixel/smart_arm/6/radians_per_encoder_tick')
    joints = ['/shoulder_pan_controller','/shoulder_pitch_controller',
              '/elbow_flex_controller', '/wrist_roll_controller', '/right_finger_controller']
    motors = ['/motor',] + ['/motor_master',]*2 + ['/motor',]*2
    
    # Initialization
    qlims = zeros((5,2))
    
    # Get parameters
    for i in range(len(joints)):
        vini = rospy.get_param(joints[i] + motors[i] + '/init')
        vmax = rospy.get_param(joints[i] + motors[i] + '/max')
        vmin = rospy.get_param(joints[i] + motors[i] + '/min')
        qlims[i,0] = -abs(vmin-vini)*tic2rad # minimum
        qlims[i,1] = +abs(vmax-vini)*tic2rad # maximum
    
    return qlims
    
    
    
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
        
    
    
    def fkine(self,qs):
        '''
        Returns the forward kinematics matrix of the whole arm by specifying
        a list of joint positions.
        '''
        assert len(qs)==4
        val = eye(4)
        for i in range(4):
            val = dot(val, self.links[i].tr(qs[i]))
        return val
        
        
    def ikine(self,xyz):
        '''
        Returns the inverse kinematics (list of joint positions) to achieve a
        (x,y,z) position on the end effector.
        '''
        x,y,z = xyz
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
        # if more than one solution, take the nearest to curent one
        return [q1,q2,q3,q4]
    
    
    
    def grab(self,auto=True):
        """
        Grabs or ungrabs depending on the joint state.
        """
        # Check condition
        if abs(joints_curr[4]-qlims[4,0]) < 0.1 and auto:
            val = +1 # close
        else:
            val = -1 # open
            
        # Publish to controller
        pub_move[4].publish(Float64(val))
        print 'published = ',val
            
        # Wait until finish movement
        while sum(joints_move) > 0:
            sleep(0.1)
            
        return 1
        
    
    
    def move_all(self,qs):
        '''
        Moves the arm to the specified joint positions.
        '''
        assert isinstance(qs,list)
        assert len(qs) == 4
        
        # Send movement commands to the smart_arm_controller (not the hand)
        for i in range(4):
            pub_move[i].publish(Float64(qs[i]))
            
        # Wait until finish movement
        while sum(joints_move) > 0:
            sleep(0.1)
        
        # Check if the goal is reached
        cond = abs( array(joints_curr[:4]) - array(qs) ) < 0.05
        if sum(cond) == 4:
            return 1
        else:
            return 0
            
            
            
    def move_joint(i,q):
        '''
        Moves the specified joint "i" to the specified position "q".
        '''
        
        
        
    def move_xyz(self,xyz):
        '''
        Moves the arm to specified (x,y,z) position of the end effector, using
        the inverse kinematics solution.
        '''
        qs = self.ikine(xyz)
        return self.move_all(qs)



################################################################################
# Default values
qlims = array([[-1.22207136,1.22207136],[-1.04822021,1.97372195],[-1.88679637,1.97372195],
               [-2.61288061,2.61799388],[-0.24032366,0.84368943]])

# Control of the joint status initialization
joints_move = [False,]*5
joints_load = [False,]*5
joints_curr = [0,]*5


                    
################################################################################
####################            MAIN            ################################
################################################################################
if __name__ == '__main__':
    # Initialize ROS node
    rospy.init_node('smart_arm_server')#, anonymous=True)
    rospy.loginfo('smart_arm_server initialized')
    
    # Publishers
    ## To controller
    pub_j1 = rospy.Publisher("/shoulder_pan_controller/command",   Float64)
    pub_j2 = rospy.Publisher("/shoulder_pitch_controller/command", Float64)
    pub_j3 = rospy.Publisher("/elbow_flex_controller/command",     Float64)
    pub_j4 = rospy.Publisher("/wrist_roll_controller/command",     Float64)
    pub_j5 = rospy.Publisher("/right_finger_controller/command",   Float64)
    pub_move = [pub_j1, pub_j2, pub_j3, pub_j4, pub_j5]
        
    # Subscribers
    ## From controller
    stat_j1 = rospy.Subscriber('/shoulder_pan_controller/state',   JointState, check_j1)
    stat_j2 = rospy.Subscriber('/shoulder_pitch_controller/state', JointState, check_j2)
    stat_j3 = rospy.Subscriber('/elbow_flex_controller/state',     JointState, check_j3)
    stat_j4 = rospy.Subscriber('/wrist_roll_controller/state',     JointState, check_j4)
    stat_j5 = rospy.Subscriber('/right_finger_controller/state',   JointState, check_j5)
    
    # Services
    srv_values = rospy.Service('get_arm_srv', SmartArmService, ask_server)

    # Get controller limits
    try:
        qlims = get_arm_limits()
        print '# Readed limits from controller:\n', qlims, '\n'
    except: # arm controller not initialized
        print '# Error: No controller found.\nLoading standard joint limits' # TODO:change to rosdebug info
        qlims = array([[-1.22207136,1.22207136],[-1.04822021,1.97372195],[-1.88679637,1.97372195],
                       [-2.61288061,2.61799388],[-0.24032366,0.84368943]])
                       
    # Sizes obtained with DH method TODO: check measures
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

    # Forward kinematics
    print '# Forward kinematics test:\n', arm.fkine([0,0,0,0]), '\n'
    #print arm.ikine(0.1,0.1,0.1)
        
    # Continue execution forever
    rospy.spin()
