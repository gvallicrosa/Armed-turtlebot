#!/usr/bin/env python
# Debugger
import ipdb

# Qt
import sys
import ui.ui_interface as ui_interface
from PyQt4 import QtCore, QtGui

# ROS
## ROS basics
import roslib
roslib.load_manifest('smart_arm_node')
import rospy
## Messages and services
from std_msgs.msg import Float64
from smart_arm_node.srv import SmartArmService

# Math
from numpy import array

# Smart arm server
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
          0 : joint status
        1-5 : move a single joint
          6 : move the four first joints to desired joint angles
          7 : move the four first joints to desired (x,y,z) using IK solver
          8 : grab/ungrab with the hand
    Data should be provided for some of the requests.
    A response according to what is accomplished is retured.
    """
    rospy.wait_for_service('get_arm_srv')
    try:
        print "=== Service request"
        print "kind: ", kind
        print "data: ", data
        use_service = rospy.ServiceProxy('get_arm_srv', SmartArmService)
        resp = use_service(kind,data)
        print "vals: ",resp.values
        print ""
        return list(resp.values)
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
        
        # Useful lists
        self.sliders = [self.slider_j1,self.slider_j2,self.slider_j3,self.slider_j4]
        self.labels = [self.stat_j1,self.stat_j2,self.stat_j3,self.stat_j4,self.stat_j5]
        
        # Update initial labels and sliders
        self.refresh_joints()
        
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
        ## Move joints button
        self.connect(self.pushButton_mjnts,  QtCore.SIGNAL("pressed()"), self.update_mjnts)
        ## Move xyz button
        self.connect(self.pushButton_xyz,    QtCore.SIGNAL("pressed()"), self.update_xyz)
        
        
        
    def update_grab(self):
        """
        Called when grab button is pressed. Calls the smart_arm_server for
        grab/ungrab with the hand.
        """
        # Request grab
        from_arm_server(8)
        # Update sliders and labels
        self.refresh_joints()

        
    
    def update_joint(self,i,plus):
        """
        Called when a joint position is changed. Sends the commands to the arm.
        """
        # Request joint status
        joints_curr = from_arm_server(0)
        # In/Decrement the position of the joint
        if plus:
            joints_curr[i] += self.increment_spin.value()
        else:
            joints_curr[i] -= self.increment_spin.value()
        # Request single joint movement
        from_arm_server(i+1,[joints_curr[i],])
        # Update sliders and labels
        self.refresh_joints()
        
        
        
    def refresh_joints(self):
        """
        Refreshes sliders and labels status, according to the real joint status.
        """
        # Request joint status
        joints_curr = from_arm_server(0)
        # Update
        for i in range(5):
            ## Update labels
            text = "%1.4f" % joints_curr[i]
            self.labels[i].setText(text)
            ## Update sliders
            if i < 4:
                self.sliders[i].setSliderPosition(get_spos(i,joints_curr[i]))
            
        
        
    def update_mjnts(self):
        """
        Called to move all joints at the same time.
        """
        # Take the values from the corresponding spinBoxes
        goal = list()
        goal.append(self.spinj1.value())
        goal.append(self.spinj2.value())
        goal.append(self.spinj3.value())
        goal.append(self.spinj4.value())
        # Ask the server to move all joints
        from_arm_server(6,goal)
        # Update sliders and labels
        self.refresh_joints()
    
    
    
    def update_xyz(self):
        """
        Called to move to an xyz position.
        """
        # Take the values from the corresponding spinBoxes
        goal = list()
        goal.append(self.spinx.value())
        goal.append(self.spiny.value())
        goal.append(self.spinz.value())
        # Ask the server to move xyz
        from_arm_server(7,goal)
        # Update sliders and labels
        self.refresh_joints()
        
        
        
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
    ## Initialize ROS node
    rospy.init_node('smart_arm_controller_gui', anonymous=True)
    rospy.loginfo('smart_arm_node initialized')
    
    # QT
    app = QtGui.QApplication(sys.argv) # create application
    form = interfaceDialog()           # create dialog
    form.show()                        # show dialog
    app.exec_()                        # start application main loop
