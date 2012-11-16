#!/usr/bin/env python

###############################################################################
# Turtlebot Mission Control (mc) Node                                         #
#                                                                             #
# Simple three-step hybrid archtecture, using 'belief, desire,                #
# intention' (BDI) deliberation.                                              #
#                                                                             #
# 0. Create a set of beliefs e.g. 'has the turtlebot crashed?' (y/n).         #
# 1. Continuously listen for changes is the beliefs.                          #
# 2. Decide what to do next (deliberate) using current belief values.         #
# 3. Perform appropriate task.                                                #
# 4. Repeat steps 1-3 until mission is complete (or error)                    #
#                                                                             #
# The mission control (MC) node does not deal with low-level functionality    #
# of the turtlebot. This is done by the 'tasks'. The mc will decide which     #
# to run when certain preconditions are met. It does not care how tasks are   #
# imnplemented.                                                               #
#                                                                             #
# TASKS:                                                                      #
# Each task is a class derived from the 'tasks.task' base class. They should  #
# be located in the 'tasks' sub-directory. This ensures that all tasks have   #
# a common interface. Tasks use services provided by the hardware controller  #
# nodes. For example: the 'gotoTarget' task uses the low-level services       #
# provided by the 'motionControl' node in order to move the turtlebot.        #
#                                                                             #
# CONTROLLERS:                                                                #
# All hardware control is performed by 'controllers'. All controllers provide #
# services which allow tasks to uses their capabilities. Some tasks may need  #
# to access multiple pieces of hardware. Controllers are just ROS nodes.      #
#                                                                             #
# E.g. motionController --> move the roomba, provide access to odometry etc.  #
# E.g. smart_arm_server --> move the arm, provide torque information etc.     #
# E.g. VISPcontroller (?) --> provide image processing capabilites etc.       #
#                                                                             #
# Controller nodes also publish messages which the MC will listen for e.g.    #
# 'target located' or 'roomba crashed into object'.                           #
###############################################################################

# Python modules

# ROS modules
import roslib
#roslib.load_manifest('mc')
import rospy

# Custom modules
import tasks

class mc:
    """Mission control class, based on hybrid BDI architecture."""

###############################################################################

    def __init__(self):
        """Initialize local varaiables etc."""
        print("* Initializing mission control node...")

        self.beliefs = {
                        'targetLocated': 0,
                        'targetCentred': 0,
                        'atTarget': 0,
                        'crashed': 0,
                        'cameraCalibrated': 0,
                        'nodesOnline': 0,
                       }

        self.currentTask

###############################################################################

    def launch(self):
        """Launch the mission control logic."""
        print("* Launching mission control logic...")

        # 1. Continuously update beliefs
        self.listen()

        # Loop until the mission is completed or until an error occurs.
        while(not rospy.is_shutdown()):

            # 2. Deliberate / decide what to do next.
            self.deliberate()

            # 3. Act.
            self.act()
            break

###############################################################################

    def listen(self)
        """Listen for changes in beliefs."""
        rospy.Subscriber("belief_crashed", data, self.updateBelief)
        rospy.Subscriber("belief_nodesOnline", data, self.updateBelief)
        rospy.Subscriber("belief_cameraCalibrated", data, self.updateBelief)
        rospy.Subscriber("belief_targetLocated", data, self.updateBelief)
        rospy.Subscriber("belief_targetCentred", data, self.updateBelief)
        rospy.Subscriber("belief_atTarget", data, self.updateBelief)
        rospy.spin()

###############################################################################

    def updateBelief(self, data):
        belief = data.belief
        value = data.value
        # Has the belief changed?
        if self.beliefs[belief] != value:
            # Update our records.
            self.beliefs[belief] = value
            print("* Updated belief: '" + str(belief) + "' = " + str(value))

###############################################################################

    def deliberate(self):
        """ Use the current values in self.beliefs to decide which
            task to perform next."""
        print("* Deliberating...")

        # The following list of rules are ordered by importance.
        # I.e. for any rule to be applied, all of the rules above
        # it must be applied first. For example, before trying to locate the target,
        # we must check that all nodes are running. Before grasping the
        # target, we must check that we are at the target.

        #######################################################################
        # Rule 1: Are all of the ROS nodes online?
        if(self.beliefs[nodesOnline] == 0)
            # Try to fix downed nodes.
            self.currentTask = tasks.fixNodes()
            return)

        #######################################################################
        # Rule 2: Has the turtlebot crashed into something?
        if(self.beliefs[crashed] == 1):
            # Try to reorient the turtlebot.
            self.currentTask = tasks.handleCrash()
            return

        #######################################################################
        # Rule 3: Is the camera calibrated?
        if(self.beliefs[cameraCalibrated] == 0):
            # Calibrate the camera.
            self.currentTask = tasks.calibrateCamera()
            return

        #######################################################################
        # Rule 4: Has the target been located?
        if(self.beliefs[targetLocated] == 0):
            # Calibrate the camera.
            self.currentTask = tasks.locateTarget()
            return

        #######################################################################
        # Rule 5: Is the target in the centre of the camera?
        if(self.beliefs[targetCentred] == 0):
            # Calibrate the camera.
            self.currentTask = tasks.centreTarget()
            return

        #######################################################################
        # Rule 6: Is the turtlebot at the target?
        if(self.beliefs[atTarget] == 0):
            # Drive to the target.
            self.currentTask = tasks.gotoTarget()
            return

###############################################################################

    def act(self):
        """Carry out the next task."""
        print("* Taking action...")
        self.currentTask.start()
        print("* Action complete.")

###############################################################################

# Module entry point.
if __name__ == "__main__":

    # Initialize ROS node.
    rospy.init_node("mc")

    # Instantiate mc class.
    m = mc()

    # Launch mc logic.
    m.launch()
