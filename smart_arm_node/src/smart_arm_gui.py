#!/usr/bin/env python
import ipdb

# Qt basics
import sys
import ui.ui_interface as ui_interface
from PyQt4 import QtCore, QtGui

# ROS basics
import roslib
roslib.load_manifest('smart_arm_node')
import rospy

# Messages and services
from std_msgs.msg import Float64
from smart_arm_node.srv import SmartArmService

# Math
from numpy import array

# Data
from smart_arm_server import qlims



################################################################################
def get_spos(link,val):
    """
    Returns the position of the slider according to the real position of the
    joint and its defined limits.
    """
    return round((val-qlims[link,0])/(qlims[link,1]-qlims[link,0])*100)
    
    

################################################################################
def from_arm_server(kind,data=[0,]):
    """
    Function to request services from the smart_arm_server.
    """
    rospy.wait_for_service('get_arm_srv')
    try:
        use_service = rospy.ServiceProxy('get_arm_srv', SmartArmService)
        resp = use_service(kind,data)
        return array(resp.values)
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e
    
    
    
################################################################################
class interfaceDialog(QtGui.QDialog, ui_interface.Ui_Dialog):
    def __init__(self, parent = None):
        """
        Constructor for the dialog.
        """
        # Basic inits
        super(interfaceDialog, self).__init__(parent)
        self.setupUi(self)
        
        # Variables
        self.grabbing = False
        joints_curr = from_arm_server(1) # request joint status
        print joints_curr
        
        # Set current position of sliders
        self.sliders = [self.slider_j1,self.slider_j2,self.slider_j3,self.slider_j4]
        for i in range(4):
            self.sliders[i].setSliderPosition(get_spos(i,joints_curr[i]))
        
        # Connections
        ## Minus buttons
        self.connect(self.pushButton_j1min,  QtCore.SIGNAL("pressed()"), self.update_j1min)
        self.connect(self.pushButton_j2min,  QtCore.SIGNAL("pressed()"), self.update_j2min)
        self.connect(self.pushButton_j3min,  QtCore.SIGNAL("pressed()"), self.update_j3min)
        self.connect(self.pushButton_j4min,  QtCore.SIGNAL("pressed()"), self.update_j4min)
        ## Plus buttons
        self.connect(self.pushButton_j1plus, QtCore.SIGNAL("pressed()"), self.update_j1plus)
        self.connect(self.pushButton_j2plus, QtCore.SIGNAL("pressed()"), self.update_j2plus)
        self.connect(self.pushButton_j3plus, QtCore.SIGNAL("pressed()"), self.update_j3plus)
        self.connect(self.pushButton_j4plus, QtCore.SIGNAL("pressed()"), self.update_j4plus)
        ## Grab button
        self.connect(self.pushButton_grab,   QtCore.SIGNAL("pressed()"), self.update_grab)
        
        
        
    def update_grab(self):
        """
        Called when grab button is pressed. Calls the smart_arm_server for
        grab/ungrab with the hand.
        """
        from_arm_server(4) # request grab

        
    
    def update_joint(self,i,plus):
        """
        Called when a joint position is changed. Sends the commands to the arm.
        """
        joints_curr = from_arm_server(1) # request joint status
        if plus:
            joints_curr[i] += 0.1
        else:
            joints_curr[i] -= 0.1
        self.sliders[i].setSliderPosition(get_spos(i,joints_curr[i])) # update slider
        joints_curr = from_arm_server(2,joints_curr[:4]) # request joint movement
        # TODO: should check joint status to update, real positions       
        
        
        
    def update_j1min(self):
        self.update_joint(0,False)
    def update_j2min(self):
        self.update_joint(1,False)
    def update_j3min(self):
        self.update_joint(2,False)
    def update_j4min(self):
        self.update_joint(3,False)
    def update_j1plus(self):
        self.update_joint(0,True)
    def update_j2plus(self):
        self.update_joint(1,True)
    def update_j3plus(self):
        self.update_joint(2,True)
    def update_j4plus(self):
        self.update_joint(3,True)
      
            

################################################################################
####################            MAIN            ################################
################################################################################
if __name__ == "__main__":
    # ROS
    # Initialize ROS node
    rospy.init_node('smart_arm_controller_gui', anonymous=True)
    rospy.loginfo('smart_arm_node initialized')
    
    # Publishers
    ## To controller
    pub_j1 = rospy.Publisher("/shoulder_pan_controller/command",   Float64)
    pub_j2 = rospy.Publisher("/shoulder_pitch_controller/command", Float64)
    pub_j3 = rospy.Publisher("/elbow_flex_controller/command",     Float64)
    pub_j4 = rospy.Publisher("/wrist_roll_controller/command",     Float64)
    pub_j5 = rospy.Publisher("/right_finger_controller/command",   Float64)
    pub_move = [pub_j1, pub_j2, pub_j3, pub_j4, pub_j5]
    
    # Subscribers
    joints_curr = [0,]*5
    
    # QT
    app = QtGui.QApplication(sys.argv)
    form = interfaceDialog()
    form.show()
    app.exec_()
