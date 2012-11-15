Current status:

Simpifying the mission control (and motionControl) node.
The plan is to have all of this completed for Monday 19th November.

I will be working on this all day on Thursday (today), Saturday and Sunday.

Todo:

Make sure controller nodes implement 'belief_XXX' publishers for
the mission control (mc) node to listen out for. See mc.py (or below) for details.


-- James

i###############################################################################
# Turtlebot Mission Control (mc) Node                                          #
#                                                                              #
# Simple three-step hybrid archtecture, using 'belief, desire,                 #
# intention' (BDI) deliberation.                                               #
#                                                                              # 
# 0. Create a set of beliefs e.g. 'has the turtlebot crashed?' (y/n).          #
# 1. Continuously listen for changes is the beliefs.                           #
# 2. Decide what to do next (deliberate) using current belief values.          #
# 3. Perform appropriate task.                                                 #
# 4. Repeat steps 1-3 until mission is complete (or error)                     #
#                                                                              #
# The mission control (MC) node does not deal with low-level functionality     #
# of the turtlebot. This is done by the 'tasks'. The mc will decide which      #
# to run when certain preconditions are met. It does not care how tasks are    #
# imnplemented.                                                                #
#                                                                              #
# TASKS:                                                                       #
# Each task is a class derived from the 'tasks.task' base class. They should   #
# be located in the 'tasks' sub-directory. This ensures that all tasks have    #
# a common interface. Tasks use services provided by the hardware controller   #
# nodes. For example: the 'gotoTarget' task uses the low-level services        #
# provided by the 'motionControl' node in order to move the turtlebot.         #
#                                                                              # 
# CONTROLLERS:                                                                 #
# All hardware control is performed by 'controllers'. All controllers provide  #
# services which allow tasks to uses their capabilities. Some tasks may need   #
# to access multiple pieces of hardware. Controllers are just ROS nodes.       #
#                                                                              #
# E.g. motionController --> move the roomba, provide access to odometry etc.   #
# E.g. smart_arm_server --> move the arm, provide torque information etc.      #
# E.g. VISPcontroller (?) --> provide image processing capabilites etc.        #
#                                                                              #
# Controller nodes also publish messages which the MC will listen for e.g.     #
# 'target located' or 'roomba crashed into object'.                            #
################################################################################

For more detials on BDI/hybrid architecture see:
"A layered architecture using schematic plans for controlling mobile robots"
http://sedici.unlp.edu.ar/handle/10915/21769

