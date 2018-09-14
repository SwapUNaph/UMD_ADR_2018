#!/usr/bin/env python

#  --- Changelog ---
# Goal:     Input by joystick. Startup drone by publishing to the state machine. Intervene by publishing to driving.
# Status:   06/19:  Manual flight possible through joystick. Takeoff with T1, Land with T2, Emergency land at Mode A, Move camera with hat.
#           06/25:  Start of script delayed until connected to state_machine. Manual flight possible through joystick. Takeoff with T1, Land with T2, Emergency land at Mode A, Manual flight with T3, automatic flight with T4.

import rospy
from std_msgs.msg import Empty
from geometry_msgs.msg import Twist
import pygame
import signal
import sys, os
import time
from bebop_msgs.msg import Ardrone3PilotingStateFlyingStateChanged
from std_msgs.msg import Bool, Int32


def signal_handler(signal, frame):
    sys.exit(0)


def publish_cmd(x,y,z,r):
    msg = Twist()  # [0,0,0],[0,pitch,yaw])
    msg.linear.x = x
    msg.linear.y = y
    msg.linear.z = z
    msg.angular.z = r
    cmd_vel_pub.publish(msg)


def publish_status(st):
    pub = rospy.Publisher('/bebop/' + st, Empty, queue_size=1, latch=True)

    msg = Empty()
    pub.publish(msg)
    pub.publish(msg)
    print('publish to /bebop/' + st)
    rospy.loginfo('publish to /bebop/' + st)


def callback_state_auto_changed(data):
    # used for initialization between jetson and ground
    global state_auto
    state_auto = data.data


def autonomy_pub(bool):
    pub = rospy.Publisher('/auto/autonomous_driving', Bool, queue_size=1, latch=True)

    pub.publish(bool)
    print('autonomy_active: ' + str(bool))
    rospy.loginfo('autonomy_active: ' + str(bool))


def list_compare(old,new):
    for i in range(len(old)):
        new[i] = new[i] - old[i]
    return new


def callback_state_bebop_changed(data):
    global state_bebop
    state_bebop = data.state


if __name__ == '__main__':
    # Enable killing the script with Ctrl+C.
    signal.signal(signal.SIGINT, signal_handler)

    rospy.init_node('ground_output', anonymous=True)

    # Initialize joystick and define loop frequency.
    pygame.init()
    pygame.joystick.init()
    clock = pygame.time.Clock()

    # Global variables for autonomy mode, and the status of the drone and the state machine
    autonomy_active = False
    flight_command_active = False
    state_bebop = 0
    state_auto = None

    rospy.Subscriber("/bebop/states/ardrone3/PilotingState/FlyingStateChanged", Ardrone3PilotingStateFlyingStateChanged,
                     callback_state_bebop_changed)
    rospy.Subscriber("/auto/state_auto", Int32, callback_state_auto_changed)

    # create a global publisher for driving manually
    cmd_vel_pub = rospy.Publisher('/bebop/cmd_vel', Twist, queue_size=1, latch=True)
    state_auto_pub = rospy.Publisher('/auto/state_auto', Int32, queue_size=1, latch=True)
    publisher_emergency_shutdown = rospy.Publisher("/auto/emergency_shutdown", Empty, queue_size=1, latch=True)

    # allow starting gcs without joystick
    start_without_joystick = False
    if start_without_joystick:
        while state_auto is None:
            rospy.loginfo("waiting None")
            time.sleep(1)
        while state_auto == 0:
            rospy.loginfo("waiting 0")
            state_auto_pub.publish(1)
            time.sleep(1)
        while state_auto == 1:
            rospy.loginfo("waiting 1")
            time.sleep(1)

            state_auto_pub.publish(2)

        print("GCS communicating")
        rospy.loginfo("GCS communicating")

        while True:
            rospy.spin()

    # Verify that there is exactly one joystick connected.  Then initialize joystick.
    while pygame.joystick.get_count() != 1:
        print("Connect exactly one joystick")
        rospy.loginfo("Connect exactly one joystick")
        time.sleep(1)
    joystick = pygame.joystick.Joystick(0)
    joystick.init()

    # Initialize current button status.
    btn_status_old = [joystick.get_button(0),
                      joystick.get_button(1),
                      joystick.get_button(2),
                      joystick.get_button(3),
                      joystick.get_button(4),
                      joystick.get_button(5),
                      joystick.get_button(6),
                      joystick.get_button(7),
                      joystick.get_button(8)]
                      # joystick.get_button(9),
                      # joystick.get_button(10),
                      # joystick.get_button(11),
                      # joystick.get_button(12),
                      # joystick.get_button(13)]

    # Wait until connection between ground and air is established
    while state_auto is None:
        rospy.loginfo("waiting None")
        time.sleep(1)
    while state_auto == 0:
        rospy.loginfo("waiting 0")
        state_auto_pub.publish(1)
        time.sleep(1)
    while state_auto == 1:
        rospy.loginfo("waiting 1")
        time.sleep(1)
    state_auto_pub.publish(2)

    print("GCS communicating")
    rospy.loginfo("GCS communicating")

    if joystick.get_name() == 'Saitek AV8R Joystick':
        rospy.loginfo('found yoke')
        controller = 'yoke'
    elif joystick.get_name() == 'Xbox 360 Wireless Receiver':
        rospy.loginfo('found xbox')
        controller = 'xbox'
        dead_zone = .2
    else:
        rospy.loginfo('controller unknown')
        sys.exit()

    while True:
        # read in axis values
        # USB Joystick
        if controller == 'yoke':
            axis_roll = -joystick.get_axis(0)
            axis_pitch = -joystick.get_axis(1)
            axis_throttle = -joystick.get_axis(2)
            axis_yaw = -joystick.get_axis(3)

        # Xbox controller
        elif controller == 'xbox':
            axis_roll = -joystick.get_axis(3)
            axis_pitch = -joystick.get_axis(4)
            axis_throttle = -joystick.get_axis(1)
            axis_yaw = -joystick.get_axis(0)
            if abs(axis_roll) < dead_zone:
                axis_roll = 0.0
            if abs(axis_pitch) < dead_zone:
                axis_pitch = 0.0
            if abs(axis_throttle) < dead_zone:
                axis_throttle = 0.0
            if abs(axis_yaw) < dead_zone:
                axis_yaw = 0.0

        # process all events that have happened
        for event in pygame.event.get():
            # Possible events are: JOYAXISMOTION JOYBALLMOTION JOYBUTTONDOWN JOYBUTTONUP JOYHATMOTION
            if event.type == pygame.JOYBUTTONDOWN or event.type == pygame.JOYBUTTONUP:
                btn_status_new = [joystick.get_button(0),  # trigger, A
                                  joystick.get_button(1),  # red, B
                                  joystick.get_button(2),  # big, X
                                  joystick.get_button(3),  # tall, Y
                                  joystick.get_button(4),  # t1
                                  joystick.get_button(5),  # t2
                                  joystick.get_button(6),  # t3
                                  joystick.get_button(7),  # t4
                                  joystick.get_button(8)]  # t5
                                  # joystick.get_button(9),  # t6
                                  # joystick.get_button(10),  # t7
                                  # joystick.get_button(11),  # t8
                                  # joystick.get_button(12),  # A
                                  # joystick.get_button(13)]  # B
                # compare old an new button status
                btn_status_diff = list_compare(btn_status_old, btn_status_new)

                if event.type == pygame.JOYBUTTONDOWN:  # button was pressed
                    for i in range(len(btn_status_diff)):  # loop through all buttons
                        if btn_status_diff[i] == 1:  # button with index i was pressed
                            if (i == 4 and controller == 'yoke') or (i == 3 and controller == 'xbox'):  # takeoff
                                if not autonomy_active:  # manual flight active
                                    if state_bebop == 0:  # drone on the ground
                                        if axis_throttle == 0:  # throttle centered
                                            publish_status("takeoff")
                                            print("Takeoff")
                                            rospy.loginfo("Takeoff")
                                            flight_command_active = True
                                        else:
                                            print("center throttle: " + "%.2f" % axis_throttle)
                                            rospy.loginfo("center throttle: " + "%.2f" % axis_throttle)
                                    else:
                                        print("not on the ground")
                                        rospy.loginfo("not on the ground")
                                else:
                                    print("not in manual mode")
                                    rospy.loginfo("not in manual mode")

                            if (i == 5 and controller == 'yoke') or (i == 2 and controller == 'xbox'):  # land
                                if True: #not autonomy_active:  # manual flight active
                                    if True: #state_bebop == 2 or state_bebop == 1 or state_bebop == 3:  # takeoff, hover, flight
                                        publish_status("land")
                                        print("Landing")
                                        rospy.loginfo("Landing")
                                        flight_command_active = False
                                    else:
                                        print("not hovering or taking off or flying")
                                        rospy.loginfo("not hovering or taking off or flying")
                                else:
                                    print("not in manual mode")
                                    rospy.loginfo("not in manual mode")

                            elif (i == 6 and controller == 'yoke') or (i == 1 and controller == 'xbox'):  # enter manual mode
                                if True:  # autonomy_active:  # autonomous flight active
                                    if True:  # axis_throttle == 0:  # throttle centered
                                        publisher_emergency_shutdown.publish()
                                        rospy.loginfo("terminate other nodes")
                                        autonomy_active = False
                                        autonomy_pub(autonomy_active)
                                        flight_command_active = True
                                    else:
                                        print("center throttle: " + "%.2f" % axis_throttle)
                                        rospy.loginfo("center throttle: " + "%.2f" % axis_throttle)
                                else:
                                    print("already in manual mode")
                                    rospy.loginfo("already in manual mode")

                            elif (i == 7 and controller == 'yoke') or (i == 0 and controller == 'xbox'):  # enter automatic mode
                                if not autonomy_active:  # manual mode active
                                    if axis_throttle == 0:  # throttle centered
                                        autonomy_active = True
                                        autonomy_pub(autonomy_active)
                                        flight_command_active = False
                                    else:
                                        print("center throttle: " + "%.2f" % axis_throttle)
                                        rospy.loginfo("center throttle: " + "%.2f" % axis_throttle)
                                else:
                                    print("already in automatic mode")
                                    rospy.loginfo("already in automatic mode")

                            elif (i == 8 and controller == 'yoke') or (i == 7 and controller == 'xbox'):  # emergency
                                publish_status("reset")
                                print("Emergency")
                                rospy.loginfo("Emergency")
                                flight_command_active = False

                            btn_status_old[i] = 1  # reset this button press

                if event.type == pygame.JOYBUTTONUP:  # button was released
                    for i in range(len(btn_status_diff)):
                        if btn_status_diff[i] == -1:
                            if i == 8:  # emergency flipped back
                                print("Emergency Reset")
                                rospy.loginfo("Emergency Reset")
                            btn_status_old[i] = 0

                           # send flt_cmd every step

        # if in manual mode and hovering or flying, publish commands
        if flight_command_active: #(not autonomy_active) and (state_bebop == 2 or state_bebop == 3):
            publish_cmd(axis_pitch, axis_roll, axis_throttle, axis_yaw)
            rospy.loginfo("ground publish driving msg")
            rospy.loginfo(axis_pitch)
            rospy.loginfo(axis_roll)
            rospy.loginfo(axis_throttle)
            rospy.loginfo(axis_yaw)

        # run with 20Hz
        clock.tick(20)
