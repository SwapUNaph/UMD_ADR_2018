#!/usr/bin/env python

#  --- Changelog ---
# Goal:     Input by bebop camera and state_machine to detect that gate has been passed. Output advances state_machine
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
