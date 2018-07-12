#!/usr/bin/env python

#  --- Changelog ---
# Goal:     Input from ground or from navigation. Ground overrides navigation. Publish to drone.
# Status:   06/19: Not existing
#           06/25: Drone takes off autonomously and lands autonomously on the same location
#           06/27: Drone takes off autonomously, flies to a virtual target and lands after reaching the target

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
from bebop_auto.msg import Auto_Driving_Msg


def signal_handler(signal, frame):
    sys.exit(0)


def publish_command(x,y,z,r):
    if not rospy.is_shutdown():
        msg = Twist()  # [0,0,0],[0,pitch,yaw])
        msg.linear.x = x
        msg.linear.y = y
        msg.linear.z = z
        msg.angular.z = r
        cmd_vel_pub.publish(msg)
    else:
        rospy.loginfo("flight command not sent")


def publish_status(st):
    pub = rospy.Publisher('/bebop/' + st, Empty, queue_size=1, latch=True)
    if not rospy.is_shutdown():
        msg = Empty()
        pub.publish(msg)
    else:
        rospy.loginfo("status command not sent")


def callback_states_changed(data, args):
    # update variables
    if args == "state_auto":
        global state_auto
        state_auto = data.data
    elif args == "state_bebop":
        global state_bebop
        state_bebop = data.state


def callback_autonomous_driving(data):
    print("autonomy " + str(data))
    global autonomy_active
    autonomy_active = data.data


def callback_autonomous_drive_msg_changed(data):
    global drive_msg
    drive_msg = data


def main():
    # Enable killing the script with Ctrl+C.
    signal.signal(signal.SIGINT, signal_handler)

    rospy.init_node('driving', anonymous=True)

    # Global variables for autonomy mode, and the status of the drone and the state machine
    global autonomy_active
    global drive_msg
    autonomy_active = False
    global state_bebop
    bebop_status = 0
    global state_auto
    state_auto = -1

    # create a state machine publisher and a global command publisher
    global cmd_vel_pub
    cmd_vel_pub = rospy.Publisher('/bebop/cmd_vel', Twist, queue_size=1, latch=True)
    rospy.Subscriber("/bebop/states/ardrone3/PilotingState/FlyingStateChanged", Ardrone3PilotingStateFlyingStateChanged,
                     callback_states_changed, "state_bebop")
    rospy.Subscriber("/auto/state_auto", Int32, callback_states_changed, "state_auto")
    rospy.Subscriber("/auto/autonomous_driving", Bool, callback_autonomous_driving)
    rospy.Subscriber("/auto/auto_drive", Auto_Driving_Msg, callback_autonomous_drive_msg_changed)

    # run with 20Hz
    rate = rospy.Rate(20)

    rospy.loginfo("ready")

    while True:
        rate.sleep()

        if autonomy_active:

            if state_auto == 2:
                rospy.loginfo("takeoff")
                publish_status("takeoff")
            elif state_auto == 5:
                rospy.loginfo("land")
                publish_status("land")


            publish_command(drive_msg.x, drive_msg.y, drive_msg.z, drive_msg.r)
            rospy.loginfo("fwd: " + "{:.2f}".format(drive_msg.x) + " | left: " + "{:.2f}".format(
                drive_msg.y) + " | up: " + "{:.2f}".format(drive_msg.z) + " | ccw: " + "{:.2f}".format(drive_msg.r))


        else:
            pass
            # publish_command(0,0,0,0)


if __name__ == '__main__':
    main()
