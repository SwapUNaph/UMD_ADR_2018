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

def signal_handler(signal, frame):
    sys.exit(0)

def startnode(package, executable):
    return roslaunch.core.Node(package, executable)

def cameramove(status):
    pub = rospy.Publisher('/bebop/camera_control', Twist, queue_size=10, latch=True)
    if not rospy.is_shutdown():
        msg = Twist() # [0,0,0],[0,pitch,yaw])
        msg.angular.y = status[1] # pitch
        msg.angular.z = status[0] # tilt
        #rospy.loginfo(msg)
        pub.publish(msg)
    else:
        print("camera command not sent")

def flt_cmd(x,y,z,r):
    pub = rospy.Publisher('/bebop/cmd_vel', Twist, queue_size=10, latch=True)
    if not rospy.is_shutdown():
        msg = Twist()  # [0,0,0],[0,pitch,yaw])
        msg.linear.x = x
        msg.linear.y = y
        msg.linear.z = z
        msg.angular.z = r
        #rospy.loginfo(msg)
        pub.publish(msg)
    else:
        print("flight command not sent")

def wificheck(target_ssid):
    scanoutput = check_output(["iwlist", "wlp4s0", "scan"])

    for line in scanoutput.split():
        if line.startswith("ESSID"):
            actual_ssid = line.split('"')[1]
            if actual_ssid <> target_ssid:
                quit("Wifi check failed: " + actual_ssid)
    print("Wifi check succeeded: " + actual_ssid)

def flt_st(st):
    pub = rospy.Publisher('/bebop/' + st, Empty, queue_size=10, latch=True)
    if not rospy.is_shutdown():
        print('publish to /bebop/' + st)
        msg = Empty()
        pub.publish(msg)
    else:
        print("status command not sent")

def list_compare(old,new):
    for i in range(len(old)):
        new[i] = new[i] - old[i]
    return(new)

if __name__ == '__main__':
    # wificheck("Bebop2-L385479")
    print("Wifi check disabled")
    #bebop_nodelet = startnode("bebop_tools", "bebop_nodelet_iv.launch")
    #print bebop_nodelet

    signal.signal(signal.SIGINT, signal_handler)

    rospy.init_node('joystick_publish', anonymous=True)

    pygame.init()

    # Set the width and height of the screen [width,height]
    pygame.joystick.init()
    clock = pygame.time.Clock()

    if pygame.joystick.get_count() != 1:
        quit("please connect exactly one Joystick")
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
    hat_status_old = joystick.get_hat(0)

    flt_status = "ground"

    camera_status=[0,0]
    #cameramove(camera_status)
    #print("moved")

    print("init done")
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
                            if i == 4:
                                if flt_status == "ground":
                                    #print axis_throttleL
                                    if axis_throttleL == 0:
                                        flt_status = "air"
                                        print(flt_status)
                                        flt_st("takeoff")
                                    else:
                                        print("please center throttle: " + "%.2f" % -axis_throttleL)
                                else:
                                    print("Your flight status is not ground")
                            elif i == 5:
                                if flt_status == "air":
                                    flt_status = "land"
                                    print(flt_status)
                                    flt_st("land")
                                else:
                                    print("Your flight status is not air")
                            elif i == 11:
                                print("Exit")
                                done = True
                            elif i ==12:
                                flt_status = "emergency"
                                print(flt_status)
                                flt_st("reset")

                            btn_status_old[i] = 1

                if event.type == pygame.JOYBUTTONUP:
                    for i in range(len(btn_status_diff)):
                        if btn_status_diff[i] == -1:
                            if i == 4:
                                pass
                            elif i == 12:
                                flt_status = "ground"
                                print(flt_status)
                            btn_status_old[i] = 0

                if event.type == pygame.JOYHATMOTION:
                    hat_status_new = joystick.get_hat(0)

                    if hat_status_old[0] == 0 and hat_status_new[0] == 1:
                        camera_status[0] += 5
                        camera_status[0] = min(camera_status[0], 35)
                        cameramove(camera_status) # x,y = tilt,pitch
                    if hat_status_old[0] == 0 and hat_status_new[0] == -1:
                        camera_status[0] -= 5
                        camera_status[0] = max(camera_status[0], -35)
                        cameramove(camera_status)
                    if hat_status_old[1] == 0 and hat_status_new[1] == 1:
                        camera_status[1] += 5
                        camera_status[1] = min(camera_status[1], 20)
                        cameramove(camera_status)
                    if hat_status_old[1] == 0 and hat_status_new[1] == -1:
                        camera_status[1] -= 5
                        camera_status[1] = max(camera_status[1], -85)
                        cameramove(camera_status)
                    hat_status_old = hat_status_new

                #if event.type == pygame.JOYAXISMOTION:
                #    print("axis moved")
                #if event.type == pygame.JOYBALLMOTION:
                #    print("ball moved")

            # send flt_cmd every step
            if flt_status == "air":
                flt_cmd(-axis_pitch,-axis_roll,-axis_throttleL,-axis_yaw)

            clock.tick(20)
        except rospy.ROSInterruptException:
            print("error")

    #try:
    #    print("end")
    #except rospy.ROSInterruptException:
    #    print("error")
    #    pass