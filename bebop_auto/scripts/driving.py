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


def flt_cmd(x,y,z,r):
    if not rospy.is_shutdown():
        msg = Twist()  # [0,0,0],[0,pitch,yaw])
        msg.linear.x = x
        msg.linear.y = y
        msg.linear.z = z
        msg.angular.z = r
        cmd_vel_pub.publish(msg)
    else:
        print("flight command not sent")


def flt_st(st):
    pub = rospy.Publisher('/bebop/' + st, Empty, queue_size=1, latch=True)
    if not rospy.is_shutdown():
        print('publish to /bebop/' + st)
        msg = Empty()
        pub.publish(msg)
    else:
        print("status command not sent")


def callback_state_machine_changed(data):
    global current_state
    current_state = data.data


def list_compare(old,new):
    for i in range(len(old)):
        new[i] = new[i] - old[i]
    return(new)


def callback_autonomous_driving(data):
    global autonomy_active
    autonomy_active = data.data


def callback_flying_state_changed(data):
    global flt_status
    flt_status = data.state
    if flt_status == 0:
        print("flt_st: 0 - landed")
    elif flt_status == 1:
        print("flt_st: 1 - takeoff")
    elif flt_status == 2:
        print("flt_st: 2 - hover")
    elif flt_status == 3:
        print("flt_st: 3 - flying")
    elif flt_status == 4:
        print("flt_st: 4 - landing")
    elif flt_status == 5:
        print("flt_st: 5 - emergency")
    elif flt_status == 6:
        print("flt_st: 6 - usertakeoff")
    elif flt_status == 7:
        print("flt_st: 7 - motor ramping")
    elif flt_status == 8:
        print("flt_st: 8 - defect: emergency landing")


if __name__ == '__main__':

    signal.signal(signal.SIGINT, signal_handler)

    rospy.init_node('driving', anonymous=True)

    autonomy_active = None
    flt_status = -1
    current_state = -1

    jetson_online = False

    state_publisher = rospy.Publisher("/auto/state_machine", Int32, queue_size=3, latch=True)
    rospy.Subscriber("/bebop/states/ardrone3/PilotingState/FlyingStateChanged", Ardrone3PilotingStateFlyingStateChanged,
                     callback_flying_state_changed)
    rospy.Subscriber("/auto/state_machine", Int32, callback_state_machine_changed)
    rospy.Subscriber("/auto/autonomous_driving", Bool, callback_autonomous_driving)

    cmd_vel_pub = rospy.Publisher('/bebop/cmd_vel', Twist, queue_size=1, latch=True)
    rate = rospy.Rate(20)

    while current_state <= 1:
        print("stuck " + str(current_state))
        time.sleep(0.5)

    while True:
        if autonomy_active:
            print("autonomous mode")
            if current_state == 2:
                flt_st("takeoff")
                state_publisher.publish(3)
            #if current_state == 4:
            #    #driving
            #    #flt_cmd(-axis_pitch, -axis_roll, -axis_throttleL, -axis_yaw)
            #    print("autonomous landing")
            #    flt_st("reset")
            if current_state == 4:
                flt_st("land")
                state_publisher.publish(5)
        else:
            print("manual mode")
            pass
        rate.sleep()


    #try:
    #    print("end")
    #except rospy.ROSInterruptException:
    #    print("error")
    #    pass