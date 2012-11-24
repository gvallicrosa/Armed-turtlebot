#!/usr/bin/env python
#from __future__ import division

# Basics
import ipdb
import roslib
roslib.load_manifest('smart_arm_node')
import rospy

# ROS messages
from std_msgs.msg import Float64, Bool, Float64MultiArray
from dynamixel_msgs.msg import JointState, MotorStateList
from smart_arm_node.srv import SmartArmService, SmartArmServiceResponse
import tf

# Maths
from numpy import pi, arctan2, cos, sin, array, sqrt, arccos, eye, dot, zeros

# Time
from time import sleep



################################################################################
def serviceHandler(req):
    """
    Accepts external requests about status and movements, depending on the
    'req.what' value.
          0 : joint status
        1-5 : move a single joint
          6 : move the four first joints to desired joint angles
          7 : move the four first joints to desired (x,y,z) using IK solver
          8 : grab/ungrab with the hand
          9 : ask for (x,y,z) position of end effector
         10 : go to home position
    Data should be provided for some of the requests.
    A response according to what is accomplished is retured.
    """
    if req.what == 0:
        # joint status
        ans = list(joints_curr)
    elif req.what in range(1,6):
        # move specified joint
        ans = [arm.move_joint(req.what,req.data[0]),]
    elif req.what == 6:
        # move radians
        qs = list(req.data)
        ans = [arm.move_all(qs),]
    elif req.what == 7:
        # move xyz
        xyz = list(req.data)
        ans = [arm.move_xyz(xyz),]
    elif req.what == 8:
        # grab/ungrab
        ans = [arm.grab(),]
    elif req.what == 9:
        # position status
        ans = arm.fkine(joints_curr, True)[:3,3]
    elif req.what == 10:
        # go home
#        home_pos = [,,,]
        ans = [arm.move_all(home_pos),]
    else:
        # no valid request
        rospy.logerr('No valid service request to smart_arm_server')
        ans = [0,]

    return SmartArmServiceResponse(ans)
        


################################################################################
def angle_wrap(a):
    '''
    Retruns the angle a normalized between -pi and pi. Values acceptable by the
    motor controllers.
    '''
    if a > pi :
        a -= 2*pi
    elif a < -pi :    
        a += 2*pi
    return a
    
    
    
################################################################################
def check_joint(msg, i):
    '''
    Checks if a joint is still moving and also if the torque is too big and the
    movement should be stopped.
    '''
    # Global variables for storage
    joints_move[i-1] = msg.is_moving       # joint moving?
    joints_load[i-1] = abs(msg.load) > 0.8 # joint overloaded?
    joints_curr[i-1] = msg.current_pos     # current position
    
    # Publish transformations for RViz of the joints (1-4), not the hand (5)
    if i < 5:
        # Get transformation between the joint and previous joint
        parent = "joint%s" % str(i-1)
        child  = "joint%s" % str(i)
        trans  = arm.links[i-1].tr(joints_curr[i-1],True)
        # Broadcast tf [translation(x,y,z), rotation(x,y,z,w), time, child, parent]
        tfbr.sendTransform(tf.transformations.translation_from_matrix(trans),
                           tf.transformations.quaternion_from_matrix(trans),
                           rospy.Time.now(),
                           child,
                           parent)
    
    # To GUI
    pub_pos[i - 1].publish(Float64(joints_curr[i - 1]))
    
    # Check joint for overload (only hand)
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
    
    def __init__(self,theta,d,a,alpha,offset,real):
        '''
        Constructor of the class link.
        theta,d,a,alpha : defined by DH method.
        offset          : difference between the real zero of the joint motors
                          and the theoric zero.
        real            : real offset in the real motors respect to the design
                          positions
        '''
        self.theta = theta
        self.d = d
        self.a = a
        self.alpha = alpha
        self.offset = offset
        self.real = real
        
        
        
    def tr(self,q=0,real=False):
        '''
        Returns the forward kinematics matrix of the link at the especified
        theta (q) value. If real is specified, the real motor offsets are taken
        into account.
        '''
        q += self.offset # correct offset
        if real:
            q -= self.real    
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
    Class to handle the whole arm of the robot with forward kinematics and
    inverse kinematics.
    '''
    
    def __init__(self,links):
        '''
        Constructor of the arm with the specified list of links.
        '''
        assert isinstance(links,list)
        self.links = links
        
    
    # ARM INFO #################################################################
    def fkine(self, qs, real=False):
        '''
        Returns the forward kinematics matrix for all joints
        '''
        assert len(qs)>=4
        val = eye(4)
        for i in range(4):
            val = dot(val, self.links[i].tr(qs[i], real))
        return val


        
    def ikine(self,xyz):
        '''
        Returns the inverse kinematics (list of joint positions) to achieve a
        (x,y,z) position on the end effector.
        '''
        # Input/output variables
        x,y,z = xyz          # goal position
        sols = zeros((4,5))  # 4 solutions obtained with IK
        
        # Inverse kinematics
        ## Solution 1
        ### Aux
        e = sqrt(x**2 + y**2)
        f = sqrt((e-a1)**2 + (d1-z)**2)
        g = sqrt(d4**2 + a3**2)
        h = arctan2(d1-z,e-a1)
        i = arctan2(a3,d4)
        j = arccos(-(f**2 - a2**2 - g**2)/(2*a2*g)) # always [0,pi]
        k = arctan2(g*sin(pi-j), a2+g*cos(pi-j))
        ### Joints
        q1 = arctan2(y,x)
        q2 = h + k
        q3 = pi - j + i
        sols[0,0] = q1
        sols[0,1] = q2
        sols[0,2] = q3
        
        ## Solution 2
        q1 = arctan2(y,x)
        q2 = -(k - h)
        q3 = -(pi-j) + i
        sols[1,0] = q1
        sols[1,1] = q2
        sols[1,2] = q3
        
        ## Solution 3
        ### Aux
        e = sqrt(x**2 + y**2)
        f = sqrt((e+a1)**2 + (d1-z)**2)
        g = sqrt(d4**2 + a3**2)
        h = arctan2(d1-z,e+a1)
        i = arctan2(a3,d4)
        j = arccos(-(f**2 - a2**2 - g**2)/(2*a2*g)) # always [0,pi]
        k = arctan2(g*sin(pi-j), a2+g*cos(pi-j))
        ### Joints
        q1 = arctan2(y,x) + pi
        q2 = pi - h - k
        q3 = -(pi - j - i)
        sols[2,0] = q1
        sols[2,1] = q2
        sols[2,2] = q3
        
        ## Solution 4
        q1 = arctan2(y,x) + pi
        q2 = pi + k - h
        q3 = pi - j + i
        sols[3,0] = q1
        sols[3,1] = q2
        sols[3,2] = q3
        
        # Check solutions
        print 'Target: %s, %s, %s' % (x,y,z)
        for i in range(4):        # for all solutions
        
            ## Check solution with FK
            trans = self.fkine(sols[i,:4])[:3,3]
            err = sqrt( sum( ( trans-array([x,y,z]) )**2 ) )
            if err > 0.01: 
                sols[i,4] = 1     # no valid solution
                
            ## Some output   
            print 'Solution %s: ' % i, trans
            print 'Error: ', sqrt( sum( ( trans-array([x,y,z]) )**2 ) )
            print 'Joints: ', sols[i,0:4]
        
        print 'found %s solutions in design space' % sum(sols[:,4]==0)
        return sols
    
    
    
    # ARM MOVEMENTS ############################################################
    def grab(self,auto=True):
        """
        Grabs or ungrabs depending on the joint state.
        """
        # TODO: check better the movement
        # Check condition
        if abs(joints_curr[4]-qlims[4,0]) < 0.1 and auto:
            val = +1 # close
        else:
            val = -1 # open
            
        # Publish to controller
        pub_move[4].publish(Float64(val))
            
        # Wait until movement end
        while sum(joints_move) > 0:
            sleep(0.1)
            
        return 1
        
    
    
    def move_all(self,qs):
        '''
        Moves the arm to the specified joint positions.
        '''
        # Send movement commands to the smart_arm_controller one by one
        ans = list()
        for i in range(4):
            ans.append(self.move_joint(i+1,qs[i]))
        return ans
            
            
            
    def move_joint(self,i,q):
        '''
        Moves the specified joint "i" to the specified position "q".
        '''
        # Publish the movement to the joint
        pub_move[i-1].publish(Float64(q))#+self.links[i-1].real))
        
        # Wait until finish movement
        while sum(joints_move) > 0:
            sleep(0.1)
        print "Movement end"
        
        # Check if the goal is reached
        cond = abs( array(joints_curr[:4]) - array(q) ) < 0.05
        if sum(cond) == 4:
            return 1
        else:
            return 0
        
        
        
    def move_xyz(self,xyz):
        '''
        Moves the arm to specified (x,y,z) position of the end effector, using
        the inverse kinematics solution.
        '''
        # Solutions in design space
        sols = self.ikine(xyz)
        for i in range(4):     # solution
            for j in range(4): # joint
                ## Convert to real space
                sols[i, j] += self.links[j].real
                sols[i, j] = angle_wrap(sols[i, j])
                ## Check solution with joint limits
                if not( min(qlims[j,:]) <= sols[i,j] <= max(qlims[j,:]) ):
                    sols[i,4] = 1 # no valid solution
                    print "Not in joint %s limits: %s" % (j, qlims[j,:])
                    break
        # Recheck number of solutions
        print 'Solutions in real space: ', sum(sols[:,4]==0)
        if sum(sols[:,4]==0) > 0:
            reachable = True
        else:
            reachable = False
        # If more than one solution, take the nearest to curent one
        if reachable:
            valid = list()
            for i in [1,3,0,2]:
                if sols[i,4] == 0:
                    valid.append(sols[i,:4])
    #        ipdb.set_trace()
            #TODO: check offset interaction here!!!!
            return self.move_all(valid[0])
        else:
            return 0
        
        
    # TF BROADCASTIGN ##########################################################
    def tf(self,i,q=0,pub=False):
        '''
        Gets the transformation from one joint. Also publishes the
        transformation if it is specified.
        '''
        trans = self.links[i-1].tr(q)
        
        if pub:
            parent = "joint%s" % str(i-1)
            child  = "joint%s" % str(i)
            # Broadcast tf [translation (x,y,z), rotation (x,y,z,w), time, child, parent]
            tfbr.sendTransform(tf.transformations.translation_from_matrix(trans),
                               tf.transformations.quaternion_from_matrix(trans),
                               rospy.Time.now(),
                               child,
                               parent)
        return trans



    def tf_all(self,qs):
        '''
        Publishes the transformations for all joints.
        '''
        for i in range(4):
            self.tf(i+1,qs[i],True)



################################################################################
# Default values
qlims = array([[-2.60265407, 2.59754080], [-1.53909406, 1.84589021],
               [-1.84589021, 1.77941771], [-1.19650501, 2.59754080],
               [-0.80789655, 0.18919096]])

# Control of the joint status initialization
joints_move = [False,]*5
joints_load = [False,]*5
joints_curr = [0,]*5


                    
################################################################################
####################            MAIN            ################################
################################################################################
if __name__ == '__main__':
    # Initialize ROS node
    rospy.init_node('smart_arm_server', log_level=rospy.INFO)
#    rospy.init_node('smart_arm_server', log_level=rospy.DEBUG)
    rospy.logdebug('Initializing smart_arm_server node...')
    
    # Publishers
    rospy.logdebug('Initializing publishers...')
    ## To controller
    pub_j1 = rospy.Publisher("/shoulder_pan_controller/command",   Float64)
    pub_j2 = rospy.Publisher("/shoulder_pitch_controller/command", Float64)
    pub_j3 = rospy.Publisher("/elbow_flex_controller/command",     Float64)
    pub_j4 = rospy.Publisher("/wrist_roll_controller/command",     Float64)
    pub_j5 = rospy.Publisher("/right_finger_controller/command",   Float64)
    pub_move = [pub_j1, pub_j2, pub_j3, pub_j4, pub_j5]
    ## To GUI
    pos_j1 = rospy.Publisher("/arm_server/j1", Float64)
    pos_j2 = rospy.Publisher("/arm_server/j2", Float64)
    pos_j3 = rospy.Publisher("/arm_server/j3", Float64)
    pos_j4 = rospy.Publisher("/arm_server/j4", Float64)
    pos_j5 = rospy.Publisher("/arm_server/j5", Float64)
    pub_pos = [pos_j1, pos_j2, pos_j3, pos_j4, pos_j5]
    ## Transform broadcaster
    tfbr = tf.TransformBroadcaster()
        
    # Subscribers
    rospy.logdebug('Initializing subscribers...')
    ## From controller
    stat_j1 = rospy.Subscriber('/shoulder_pan_controller/state',   JointState, check_joint, 1)
    stat_j2 = rospy.Subscriber('/shoulder_pitch_controller/state', JointState, check_joint, 2)
    stat_j3 = rospy.Subscriber('/elbow_flex_controller/state',     JointState, check_joint, 3)
    stat_j4 = rospy.Subscriber('/wrist_roll_controller/state',     JointState, check_joint, 4)
    stat_j5 = rospy.Subscriber('/right_finger_controller/state',   JointState, check_joint, 5)
    
    # Services
    rospy.logdebug('Initializing services...')
    srv_values = rospy.Service('/arm_server/services', SmartArmService, serviceHandler)

    # Get controller limits
    rospy.logdebug('Reading joint limits from controller...')
    try:
        qlims = get_arm_limits()
        rospy.loginfo((' Readed limits from controller:\n%s' % qlims))
    except: # arm controller not initialized
        qlims = array([[-2.60265407, 2.59754080], [-1.53909406, 1.84589021],
                       [-1.84589021, 1.77941771], [-1.19650501, 2.59754080],
                       [-0.80789655, 0.18919096]])
        rospy.logerr(('No controller found. Loading standard joint limits...\n%s' % qlims))

                       
    # Sizes obtained with DH method
    rospy.logdebug('Setting up arm...')
    a1 = 0.051
    a2 = 0.174
    a3 = 0.023
    d1 = 0.14
    d4 = 0.2#0.16
    off2 = +0.680064816
    off3 = -0.071585770
    off4 = +0.035792885

    # Link creation
    links = list()
    links.append( Link(    0,  d1, a1, -pi/2,     0,    0) )
    links.append( Link(    0,   0, a2,    pi,     0, off2) ) # offset=-45
    links.append( Link(-pi/2,   0, a3,  pi/2, -pi/2, off3) )
    links.append( Link(   pi, -d4,  0,    pi,    pi, off4) )
    
    # Robot creation
    arm = Arm(links)
        
    # Continue execution forever
    rospy.loginfo('Node smart_arm_server initialized.')
    rospy.spin()
