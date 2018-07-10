#!/bin/bash

# rqt_gcs_gui requires rqt_gui
sudo apt-get install ros-kinetic-rqt ros-kinetic-rqt-common-plugins 

# bebop_auto requirements
sudo apt-get install ros-kinetic-robot-upstart 
sudo apt-get install ros-kinetic-rviz
sudo apt-get install ros-kinetic-robot-state-publisher
sudo apt-get install python-tf
sudo apt-get install python-pygame
sudo apt-get install ros-kinetic-tf
sudo python -m pip install https://pypi.python.org/packages/source/g/getch/getch-1.0-python2.tar.gz



# run catkin_make in workspace
sudo ~/catkin_ws/catkin_make
# close all terminals
exit
