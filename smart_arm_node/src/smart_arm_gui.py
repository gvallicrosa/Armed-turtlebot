import sys
from PyQt4 import QtCore, QtGui
from smart_arm_node import qlims, joints_curr
import ui.ui_interface as ui_interface

import roslib
roslib.load_manifest('smart_arm_node')
import rospy
from std_msgs.msg import Float64

joints_temp = joints_curr

################################################################################
def get_spos(link,val):
    return round((val-qlims[link,0])/(qlims[link,1]-qlims[link,0])*100)
    
    

################################################################################
class interfaceDialog(QtGui.QDialog, ui_interface.Ui_Dialog):
    def __init__(self, parent = None):
        super(interfaceDialog, self).__init__(parent)
        self.setupUi(self)
        
        # Variables
        self.grabbing = False
        
        # Set current position of sliders
        self.sliders = [self.slider_j1,self.slider_j2,self.slider_j3,self.slider_j4]
        for i in range(4):
            self.sliders[i].setSliderPosition(get_spos(i,joints_curr[i]))
        
        # Connections
        ## Minus buttons
        self.connect(self.pushButton_j1min, QtCore.SIGNAL("pressed()"), self.update_j1min)
        self.connect(self.pushButton_j2min, QtCore.SIGNAL("pressed()"), self.update_j2min)
        self.connect(self.pushButton_j3min, QtCore.SIGNAL("pressed()"), self.update_j3min)
        self.connect(self.pushButton_j4min, QtCore.SIGNAL("pressed()"), self.update_j4min)
        ## Plus buttons
        self.connect(self.pushButton_j1plus, QtCore.SIGNAL("pressed()"), self.update_j1plus)
        self.connect(self.pushButton_j2plus, QtCore.SIGNAL("pressed()"), self.update_j2plus)
        self.connect(self.pushButton_j3plus, QtCore.SIGNAL("pressed()"), self.update_j3plus)
        self.connect(self.pushButton_j4plus, QtCore.SIGNAL("pressed()"), self.update_j4plus)
        ## Grab button
        self.connect(self.pushButton_grab, QtCore.SIGNAL("pressed()"), self.update_grab)
        
        
        
    def update_grab(self):
        if self.grabbing:
            # ungrab
            self.grabbing = False
            self.pushButton_grab.setText('Grab (G)')
            pos = -1
        else:
            # grab
            self.grabbing = True
            self.pushButton_grab.setText('UnGrab (G)')
            pos = 1
        pub_move[4].publish(Float64(pos)) # send ros movement 
        
    
    def update_joint(self,i,plus):
        if plus:
            joints_temp[i] += 0.2
        else:
            joints_temp[i] -= 0.2
        self.sliders[i].setSliderPosition(get_spos(i,joints_temp[i])) # update slider
        pub_move[i].publish(Float64(joints_temp[i])) # send ros movement
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
      
            

## Execution of main program
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
    
    # QT
    app = QtGui.QApplication(sys.argv)
    form = interfaceDialog()
    form.show()
    app.exec_()
