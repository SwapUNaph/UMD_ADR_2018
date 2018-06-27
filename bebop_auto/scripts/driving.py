#!/usr/bin/env python

#  --- Changelog ---
# Goal:     Input from ground or from navigation. Ground overrides navigation. Publish to drone.
# Status:   06/19: Not existing
#           06/25: Drone takes off autonomously and lands autonomously on the same location

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


def callback_state_machine_changed(data):
    global state_machine
    state_machine = data.data


def callback_autonomous_driving(data):
    print("autonomy changed")
    global autonomy_active
    autonomy_active = data.data


def callback_bebop_state_changed(data):
    global bebop_status
    bebop_status = data.state


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
    global bebop_status
    bebop_status = 0
    global state_machine
    state_machine = -1

    # create a state machine publisher and a global command publisher
    global cmd_vel_pub
    cmd_vel_pub = rospy.Publisher('/bebop/cmd_vel', Twist, queue_size=1, latch=True)
    state_publisher = rospy.Publisher("/auto/state_machine", Int32, queue_size=3, latch=True)
    rospy.Subscriber("/bebop/states/ardrone3/PilotingState/FlyingStateChanged", Ardrone3PilotingStateFlyingStateChanged,
                     callback_bebop_state_changed)
    rospy.Subscriber("/auto/state_machine", Int32, callback_state_machine_changed)
    rospy.Subscriber("/auto/autonomous_driving", Bool, callback_autonomous_driving)
    rospy.Subscriber("/auto/auto_drive", Auto_Driving_Msg, callback_autonomous_drive_msg_changed)

    # run with 20Hz
    rate = rospy.Rate(20)

    # Wait until connecction between ground and air is established. Script can get stuck here
    while state_machine <= 1:
        rospy.loginfo("stuck " + str(state_machine))
        time.sleep(0.5)

    while True:
        if autonomy_active:
            if state_machine == 2:
                publish_status("takeoff")
                state_publisher.publish(3)
            if state_machine == 4:
                print(drive_msg)
                publish_command(drive_msg.x, drive_msg.y, drive_msg.z, drive_msg.r)
                rospy.loginfo("flying")
            if state_machine == 5:
                publish_status("land")
                state_publisher.publish(6)
        else:
            publish_command(0,0,0,0)
            pass
        rate.sleep()


if __name__ == '__main__':
    main()
