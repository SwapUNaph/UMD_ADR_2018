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
import sys
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


def main():
    # Enable killing the script with Ctrl+C.
    signal.signal(signal.SIGINT, signal_handler)

    rospy.init_node('ground_output', anonymous=True)

    # Initialize joystick and define loop frequency.
    pygame.init()
    pygame.joystick.init()
    clock = pygame.time.Clock()

    # Global variables for autonomy mode, and the status of the drone and the state machine
    global autonomy_active
    autonomy_active = False
    global state_bebop
    state_bebop = 0
    global state_auto
    state_auto = None

    rospy.Subscriber("/bebop/states/ardrone3/PilotingState/FlyingStateChanged", Ardrone3PilotingStateFlyingStateChanged,
                     callback_state_bebop_changed)
    rospy.Subscriber("/auto/state_auto", Int32, callback_state_auto_changed)

    # create a global publisher for driving manually
    global cmd_vel_pub
    cmd_vel_pub = rospy.Publisher('/bebop/cmd_vel', Twist, queue_size=1, latch=True)
    state_auto_pub = rospy.Publisher('/auto/state_auto', Int32, queue_size=1, latch=True)

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
                      joystick.get_button(8),
                      joystick.get_button(9),
                      joystick.get_button(10),
                      joystick.get_button(11),
                      joystick.get_button(12),
                      joystick.get_button(13)]

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

    done = False
    while not done:
        # read in axis values
        axis_roll = joystick.get_axis(0)
        axis_pitch = joystick.get_axis(1)
        axis_throttleL = joystick.get_axis(2)
        axis_yaw = joystick.get_axis(3)
        axis_throttleR = joystick.get_axis(4)

        # process all events that have happened
        for event in pygame.event.get():
            # Possible events are: JOYAXISMOTION JOYBALLMOTION JOYBUTTONDOWN JOYBUTTONUP JOYHATMOTION
            if event.type == pygame.JOYBUTTONDOWN or event.type == pygame.JOYBUTTONUP:
                btn_status_new = [joystick.get_button(0),  # trigger
                                  joystick.get_button(1),  # red
                                  joystick.get_button(2),  # big
                                  joystick.get_button(3),  # tall
                                  joystick.get_button(4),  # t1
                                  joystick.get_button(5),  # t2
                                  joystick.get_button(6),  # t3
                                  joystick.get_button(7),  # t4
                                  joystick.get_button(8),  # t5
                                  joystick.get_button(9),  # t6
                                  joystick.get_button(10),  # t7
                                  joystick.get_button(11),  # t8
                                  joystick.get_button(12),  # A
                                  joystick.get_button(13)]  # B
                # compare old an new button status
                btn_status_diff = list_compare(btn_status_old, btn_status_new)

                if event.type == pygame.JOYBUTTONDOWN:  # button was pressed
                    for i in range(len(btn_status_diff)):  # loop through all buttons
                        if btn_status_diff[i] == 1:  # button with index i was pressed
                            if i == 4:  # takeoff
                                if not autonomy_active:  # manual flight active
                                    if state_bebop == 0:  # drone on the ground
                                        if axis_throttleL == 0:  # throttle centered
                                            publish_status("takeoff")
                                            print("Takeoff")
                                            rospy.loginfo("Takeoff")
                                        else:
                                            print("center throttle: " + "%.2f" % -axis_throttleL)
                                            rospy.loginfo("center throttle: " + "%.2f" % -axis_throttleL)
                                    else:
                                        print("not on the ground")
                                        rospy.loginfo("not on the ground")
                                else:
                                    print("not in manual mode")
                                    rospy.loginfo("not in manual mode")

                            if i == 5:  # land
                                if True: #not autonomy_active:  # manual flight active
                                    if True: #state_bebop == 2 or state_bebop == 1 or state_bebop == 3:  # takeoff, hover, flight
                                        publish_status("land")
                                        print("Landing")
                                        rospy.loginfo("Landing")
                                    else:
                                        print("not hovering or taking off or flying")
                                        rospy.loginfo("not hovering or taking off or flying")
                                else:
                                    print("not in manual mode")
                                    rospy.loginfo("not in manual mode")

                            elif i == 6:  # enter manual mode
                                if autonomy_active:  # autonomous flight active
                                    if True:# axis_throttleL == 0:  # throttle centered
                                        autonomy_active = False
                                        autonomy_pub(autonomy_active)
                                    else:
                                        print("center throttle: " + "%.2f" % -axis_throttleL)
                                        rospy.loginfo("center throttle: " + "%.2f" % -axis_throttleL)
                                else:
                                    print("already in manual mode")
                                    rospy.loginfo("already in manual mode")

                            elif i == 7:  # enter automatic mode
                                if not autonomy_active:  # manual mode active
                                    if axis_throttleL == 0:  # throttle centered
                                        autonomy_active = True
                                        autonomy_pub(autonomy_active)
                                    else:
                                        print("center throttle: " + "%.2f" % -axis_throttleL)
                                        rospy.loginfo("center throttle: " + "%.2f" % -axis_throttleL)
                                else:
                                    print("already in automatic mode")
                                    rospy.loginfo("already in automatic mode")

                            elif i == 11:  # quit
                                done == True

                            elif i == 12:  # emergency
                                publish_status("reset")
                                print("Emergency")
                                rospy.loginfo("Emergency")

                            btn_status_old[i] = 1  # reset this button press

                if event.type == pygame.JOYBUTTONUP:  # button was released
                    for i in range(len(btn_status_diff)):
                        if btn_status_diff[i] == -1:
                            if i == 12:  # emergency flipped back
                                print("Emergency Reset")
                                rospy.loginfo("Emergency Reset")
                            btn_status_old[i] = 0

                           # send flt_cmd every step

        # if in manual mode and hovering or flying, publish commands
        if (not autonomy_active) and (state_bebop == 2 or state_bebop == 3):
            publish_cmd(-axis_pitch, -axis_roll, -axis_throttleL, -axis_yaw)
            rospy.loginfo("ground publish driving msg")
            rospy.loginfo(-axis_pitch)
            rospy.loginfo(-axis_roll)
            rospy.loginfo(-axis_throttleL)
            rospy.loginfo(-axis_yaw)

        # run with 20Hz
        clock.tick(20)

if __name__ == '__main__':
    main()
