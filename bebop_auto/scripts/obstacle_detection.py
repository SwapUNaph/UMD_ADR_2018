#!/usr/bin/env python

#  --- Changelog ---
# Goal:     Input stereo camera to identify obstacles in vicinity. Create a map with obstacles or a list of obstacles that feeds into path planner
# Status:   06/19: Not existing
#           06/25: Empty file

import rospy
import roslaunch
import time
from std_msgs.msg import Empty
from subprocess import check_output
from geometry_msgs.msg import Twist
import pygame
import signal
import sys
import time
from bebop_msgs.msg import Ardrone3PilotingStateFlyingStateChanged
from std_msgs.msg import Bool, Int32

