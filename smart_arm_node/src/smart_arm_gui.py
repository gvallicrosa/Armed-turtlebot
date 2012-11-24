#!/usr/bin/env python
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


###############################################################################
def get_spos(link, val):
    """
    Returns the position of the slider according to the real position of the
    joint and its defined limits.
    """
    return round((val - qlims[link, 0]) / (qlims[link, 1] - qlims[link, 0]) * 100)


################################################################################
def from_arm_server(kind, data=[0, ]):
    """
    Function to request services from the smart_arm_server.
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
    rospy.wait_for_service('/arm_server/services')
    try:
        print "=== Service request"
        print "kind: ", kind
        print "data: ", data
        use_service = rospy.ServiceProxy('/arm_server/services', SmartArmService)
        resp = use_service(kind, data)
        print "resp: ", resp.values
        print ""
        return list(resp.values)
    except rospy.ServiceException, e:
        rospy.logerr(("Service call failed: %s" % e))


################################################################################
class interfaceDialog(QtGui.QDialog, ui_interface.Ui_Dialog):
    def __init__(self, parent=None):
        """
        Constructor for the dialog.
        """
        # Basic inits
        super(interfaceDialog, self).__init__(parent)
        self.setupUi(self)

        # Useful lists
        self.jointpos = [self.spinj1, self.spinj2, self.spinj3, self.spinj4]
        self.sliders = [self.slider_j1, self.slider_j2, self.slider_j3, self.slider_j4]
        self.labels = [self.stat_j1, self.stat_j2, self.stat_j3, self.stat_j4,
                       self.stat_j5, self.stat_x, self.stat_y, self.stat_z]

        # Update initial labels and sliders
        self.refresh_joints()

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
        ## Home button
        self.connect(self.pushButton_home, QtCore.SIGNAL("pressed()"), self.update_home)
        ## Move single joint button
        self.connect(self.pushButton_mj1, QtCore.SIGNAL("pressed()"), self.update_mov_j1)
        self.connect(self.pushButton_mj2, QtCore.SIGNAL("pressed()"), self.update_mov_j2)
        self.connect(self.pushButton_mj3, QtCore.SIGNAL("pressed()"), self.update_mov_j3)
        self.connect(self.pushButton_mj4, QtCore.SIGNAL("pressed()"), self.update_mov_j4)
        ## Move all joints button
        self.connect(self.pushButton_mjnts, QtCore.SIGNAL("pressed()"), self.update_mjnts)
        ## Move xyz button
        self.connect(self.pushButton_xyz, QtCore.SIGNAL("pressed()"), self.update_xyz)

    def update_grab(self):
        """
        Called when grab button is pressed. Calls the smart_arm_server for
        grab/ungrab with the hand.
        """
        # Request grab
        from_arm_server(8)
        # Update sliders and labels
        self.refresh_joints()
        
    def update_home(self):
        """
        Moves arm to home defined position
        """
        # Request home position
        from_arm_server(10)

    def update_joint(self, i, plus):
        """
        Called when a joint position is changed. Sends the commands to the arm.
        """
        # Request joint status
#        joints_curr = from_arm_server(0)
        # In/Decrement the position of the joint
        if plus:
            joints_curr[i] += self.increment_spin.value()
        else:
            joints_curr[i] -= self.increment_spin.value()
        # Request single joint movement
        from_arm_server(i + 1, [joints_curr[i], ])
        # Update sliders and labels
        self.refresh_joints()

    def refresh_joints(self):
        """
        Refreshes sliders and labels status, according to the real joint status.
        """
        # Request joint status
#        joints_curr = from_arm_server(0)
        # Update
        for i in range(5):
            ## Update labels
            text = "%1.4f" % joints_curr[i]
            self.labels[i].setText(text)
            ## Update sliders
            if i < 4:
                self.sliders[i].setSliderPosition(get_spos(i, joints_curr[i]))
        # Request position status
        xyz = from_arm_server(9, joints_curr)
        for i in range(3):
            ## Update labels
            text = "%1.4f" % xyz[i]
            self.labels[5 + i].setText(text)
            
    def update_movejoint(self, i):
        """
        Moves one joint to the specified value in its spinBox
        """
        # Take value from spinBox
        goal = self.jointpos[i - 1].value()
        # Request movement
        from_arm_server(i, goal)

    def update_mjnts(self):
        """
        Called to move all joints at the same time.
        """
        # Take the values from the corresponding spinBoxes
        goal = list()
        for i in range(4):
            goal.append(self.jointpos[i].value())
        # Ask the server to move all joints
        from_arm_server(6, goal)
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
        from_arm_server(7, goal)
        # Update sliders and labels
        self.refresh_joints()

    def update_j1min(self):
        self.update_joint(0, False)

    def update_j2min(self):
        self.update_joint(1, False)

    def update_j3min(self):
        self.update_joint(2, False)

    def update_j4min(self):
        self.update_joint(3, False)

    def update_j1plus(self):
        self.update_joint(0, True)

    def update_j2plus(self):
        self.update_joint(1, True)

    def update_j3plus(self):
        self.update_joint(2, True)

    def update_j4plus(self):
        self.update_joint(3, True)
        
    def update_mov_j1(self):
        self.update_movejoint(1)
        
    def update_mov_j2(self):
        self.update_movejoint(2)
        
    def update_mov_j3(self):
        self.update_movejoint(3)
        
    def update_mov_j4(self):
        self.update_movejoint(4)


################################################################################
def update_stat(msg, i):
    joints_curr[i - 1] = msg.data
    if i == 1:
        form.refresh_joints()


################################################################################
####################            MAIN            ################################
################################################################################
if __name__ == "__main__":
    # Initialize ROS node
    rospy.init_node('smart_arm_controller_gui', anonymous=True)
    rospy.loginfo('smart_arm_node initialized')
    joints_curr = [0, 0, 0, 0, 0]

    # QT
    app = QtGui.QApplication(sys.argv)  # create application
    form = interfaceDialog()            # create dialog
    form.show()                         # show dialog
    
    # Subscribers
    stat_j1 = rospy.Subscriber("/arm_server/j1", Float64, update_stat, 1)
    stat_j2 = rospy.Subscriber("/arm_server/j2", Float64, update_stat, 2)
    stat_j3 = rospy.Subscriber("/arm_server/j3", Float64, update_stat, 3)
    stat_j4 = rospy.Subscriber("/arm_server/j4", Float64, update_stat, 4)
    stat_j5 = rospy.Subscriber("/arm_server/j5", Float64, update_stat, 5)
    
    # Execute application loop
    app.exec_()                         # start application main loop
