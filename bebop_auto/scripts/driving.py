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
        print("flight command not sent")


def publish_status(st):
    pub = rospy.Publisher('/bebop/' + st, Empty, queue_size=1, latch=True)
    if not rospy.is_shutdown():
        print('publish to /bebop/' + st)
        msg = Empty()
        pub.publish(msg)
    else:
        print("status command not sent")


def callback_state_machine_changed(data):
    global state_machine
    state_machine = data.data


def callback_autonomous_driving(data):
    global autonomy_active
    autonomy_active = data.data


def callback_bebop_state_changed(data):
    global bebop_status
    bebop_status = data.state
    if bebop_status == 0:
        print("flt_st: 0 - landed")
    elif bebop_status == 1:
        print("flt_st: 1 - takeoff")
    elif bebop_status == 2:
        print("flt_st: 2 - hover")
    elif bebop_status == 3:
        print("flt_st: 3 - flying")
    elif bebop_status == 4:
        print("flt_st: 4 - landing")
    elif bebop_status == 5:
        print("flt_st: 5 - emergency")
    elif bebop_status == 6:
        print("flt_st: 6 - usertakeoff")
    elif bebop_status == 7:
        print("flt_st: 7 - motor ramping")
    elif bebop_status == 8:
        print("flt_st: 8 - defect: emergency landing")


def main():
    # Enable killing the script with Ctrl+C.
    signal.signal(signal.SIGINT, signal_handler)

    rospy.init_node('driving', anonymous=True)

    # Global variables for autonomy mode, and the status of the drone and the state machine
    global autonomy_active
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

    # run with 20Hz
    rate = rospy.Rate(20)

    # Wait until connecction between ground and air is established. Script can get stuck here
    while state_machine <= 1:
        print("stuck " + str(state_machine))
        time.sleep(0.5)

    while True:
        if autonomy_active:
            if state_machine == 2:
                publish_status("takeoff")
                state_publisher.publish(3)
            #if current_state == 4:
            #    #driving
            #    #flt_cmd(-axis_pitch, -axis_roll, -axis_throttleL, -axis_yaw)
            #    print("autonomous landing")
            #    flt_st("reset")
            if state_machine == 4:
                publish_status("land")
                state_publisher.publish(5)
        else:
            pass
        rate.sleep()


if __name__ == '__main__':
    main()
