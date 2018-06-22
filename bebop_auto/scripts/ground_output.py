#!/usr/bin/env python
# license removed for brevity
import rospy
import roslaunch
import time
#from std_msgs.msg import String
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
    if data.data == 0:
        # print("I heard from the drone")
        pub = rospy.Publisher('/auto/state_machine', Int32, queue_size=5, latch=True)
        # print('I will tell her')
        pub.publish(1)
    elif data.data == 2:
        global jetson_online
        jetson_online = True

def autonomy_pub(bool):
    pub = rospy.Publisher('/auto/autonomous_driving', Bool, queue_size=1, latch=True)
    if not rospy.is_shutdown():
        print('autonomy_active: ' + str(bool))
        pub.publish(bool)
    else:
        print("status command not sent")


def list_compare(old,new):
    for i in range(len(old)):
        new[i] = new[i] - old[i]
    return(new)


def callback_flying_state_changed(data):
    global flt_status
    flt_status = data.state
    # print("flying_state_changed")
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

    rospy.init_node('ground_output', anonymous=True)

    pygame.init()

    # Set the width and height of the screen [width,height]
    pygame.joystick.init()
    clock = pygame.time.Clock()

    while pygame.joystick.get_count() != 1:
        print("Connect exactly one joystick")
        time.sleep(1)
    joystick = pygame.joystick.Joystick(0)
    joystick.init()

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

    autonomy_active = None
    flt_status = -1

    jetson_online = False

    rospy.Subscriber("/bebop/states/ardrone3/PilotingState/FlyingStateChanged", Ardrone3PilotingStateFlyingStateChanged,
                     callback_flying_state_changed)
    rospy.Subscriber("/auto/state_machine", Int32, callback_state_machine_changed)

    cmd_vel_pub = rospy.Publisher('/bebop/cmd_vel', Twist, queue_size=1, latch=True)

    print("GCS initialized")
    rospy.loginfo("GCS initialized")

    while not jetson_online:
        time.sleep(0.5)

    print("GCS communicating")
    rospy.loginfo("GCS communicating")

    done = False
    while done == False:
        try:
            axis_roll = joystick.get_axis(0)
            axis_pitch = joystick.get_axis(1)
            axis_throttleL = joystick.get_axis(2)
            axis_yaw = joystick.get_axis(3)
            axis_throttleR = joystick.get_axis(4)

            # EVENT PROCESSING STEP
            for event in pygame.event.get():  # User did something

                # Possible joystick actions: JOYAXISMOTION JOYBALLMOTION JOYBUTTONDOWN JOYBUTTONUP JOYHATMOTION
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
                    btn_status_diff = list_compare(btn_status_old, btn_status_new)

                if event.type == pygame.JOYBUTTONDOWN:
                    for i in range(len(btn_status_diff)):
                        if btn_status_diff[i]==1:
                            if i == 4:  # takeoff
                                if not autonomy_active:
                                    if flt_status == 0:
                                        if axis_throttleL == 0:
                                            flt_st("takeoff")
                                            print("Takeoff")
                                        else:
                                            print("center throttle: " + "%.2f" % -axis_throttleL)
                                    else:
                                        print("not on the ground")
                                else:
                                    print("not in manual mode")
                            if i == 5:  # land
                                if not autonomy_active:
                                    if flt_status == 2 or flt_status == 1 or flt_status == 3 or flt_status == 5:
                                        flt_st("land")
                                        print("Landing")
                                    else:
                                        print("not hovering or taking off or flying")
                                else:
                                    print("not in manual mode")

                            elif i == 8:  # set manual mode
                                if autonomy_active or autonomy_active is None:
                                    if axis_throttleL == 0:
                                        autonomy_active = False
                                        autonomy_pub(autonomy_active)
                                    else:
                                        print("center throttle: " + "%.2f" % -axis_throttleL)
                                else:
                                    print("already in manual mode")

                            elif i == 9:  # enter automatic mode
                                if not autonomy_active:
                                    if axis_throttleL == 0:
                                        autonomy_active = True
                                        autonomy_pub(autonomy_active)
                                    else:
                                        print("center throttle: " + "%.2f" % -axis_throttleL)
                                else:
                                    print("already in automatic mode")

                            elif i == 12:
                                flt_st("reset")
                                print("Emergency")

                            btn_status_old[i] = 1

                if event.type == pygame.JOYBUTTONUP:
                    for i in range(len(btn_status_diff)):
                        if btn_status_diff[i] == -1:
                            if i == 4:
                                pass
                            elif i == 12:
                                print("Emergency Reset")
                            btn_status_old[i] = 0

                           # send flt_cmd every step

            if not autonomy_active:
                flt_cmd(-axis_pitch,-axis_roll,-axis_throttleL,-axis_yaw)

            clock.tick(20)
        except rospy.ROSInterruptException:
            print("error")

    #try:
    #    print("end")
    #except rospy.ROSInterruptException:
    #    print("error")
    #    pass