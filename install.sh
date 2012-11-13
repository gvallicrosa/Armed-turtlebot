#!/bin/bash

# DEPENDENCIES
## Install dependencies
sudo apt-get install pyqt4-dev-tools python-qt4 python-qt4-devqt4-dev-tools python-rosdep

# SMART_ARM_CONTROLLER
## Move to ros folder and download the smart_arm_controller
cd ..
svn checkout http://ua-ros-pkg.googlecode.com/svn/trunk/arrg/crustcrawler_smart_arm/smart_arm_controller
rosdep install smart_arm_controller
rosmake smart_arm_controller
## Patch the controller configuration

## Change permissions to access the arm
sudo echo $'SUBSYSTEMS=="usb", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6001", GROUP="plugdev", MODE="0777"' > /etc/udev/rules.d/88-smart_arm.rules
sudo service udev restart
sudo udevadm control --reload-rules

# COMPILE VISP
cd Armed-turtlebot/visplib
mkdir build
cd build
cmake ..
make 
make install

# COMPILE GUI
cd ../..
sh smart_arm_node/src/ui/compile.sh

# COMPILE THE WHOLE STACK
rosdep install Armed-turtlebot
rosmake Armed-turtlebot
