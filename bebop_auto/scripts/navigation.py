#!/usr/bin/env python

#  --- Changelog ---
# Goal:     Input of both path planners. Find relevant waypoint depending on current position. Use visual path if possible, otherwise use blind backup. Calculate driving command based on waypoint position and orientation. Create driving commands
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


# read in position and path
# if we are close enough to the last waypoint, advance status
