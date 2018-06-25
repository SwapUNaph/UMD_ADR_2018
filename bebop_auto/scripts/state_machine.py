#!/usr/bin/env python

#  --- Changelog ---
# Goal:     Input is start signal from ground_output and gate crossing confirmation. Output is a state on a global map, the current position, the upcoming gate type and similar
# Status:   06/19:  Not existing
#           06/25:  Start delayed until connected to ground_output.
#           06/25:

from __future__ import print_function
import sys
import rospy
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Empty, Bool, Int32
from bebop_msgs.msg import Ardrone3PilotingStateFlyingStateChanged
import time


def callback_autonomous_driving(data):

    print("autonomy_active: " + str(data.data))
    rospy.loginfo("autonomy_active: " + str(data.data))


def callback_flying_state_changed(data):
    print("flying state changed")
    print("data " + str(data.state) + " and state " + str(current_state))
    if data.state == 2 and current_state == 3:  # taking off completed
        print("takeoff completed")
        state_publisher.publish(4)

    if data.state == 0 and current_state == 5:  # landing completed
        state_publisher.publish(6)

    # state_landed = 0  # Landed state
    # state_takingoff = 1  # Taking off state
    # state_hovering = 2  # Hovering / Circling (for fixed wings) state
    # state_flying = 3  # Flying state
    # state_landing = 4  # Landing state
    # state_emergency = 5  # Emergency state
    # state_usertakeoff = 6  # User take off state. Waiting for user action to take off.
    # state_motor_ramping = 7  # Motor ramping state (for fixed wings).
    # state_emergency_landing = 8  # Emergency landing state. Drone autopilot has detected defective sensor(s). Only Yaw argument in PCMD is taken into account. All others flying commands are ignored.


def callback_state_machine_changed(data):
    global current_state
    current_state = data.data

    if current_state == 0:
        pass
        # rospy.loginfo("I talked to ground")
        # print("I talked to ground")
    elif current_state == 1:
        # rospy.loginfo("I heard back from ground")
        time.sleep(0.5)
        state_publisher.publish(2)
    elif current_state == 2:
        global gcs_online
        gcs_online = True

if __name__ == '__main__':

    rospy.init_node('state_machine', anonymous=True)
    state_publisher = rospy.Publisher("/auto/state_machine", Int32, queue_size=3, latch=True)
    
    rospy.Subscriber("/auto/autonomous_driving", Bool, callback_autonomous_driving)
    rospy.Subscriber("/bebop/states/ardrone3/PilotingState/FlyingStateChanged", Ardrone3PilotingStateFlyingStateChanged, callback_flying_state_changed)
    rospy.Subscriber("/auto/state_machine", Int32, callback_state_machine_changed)

    current_state = None
    gcs_online = False
    state_publisher.publish(0)

    print("Jetson initialized")
    rospy.loginfo("Jetson initialized")

    while not gcs_online:
        time.sleep(0.5)

    print("Jetson communicating")
    rospy.loginfo("Jetson communicating")


    rospy.spin()

    print("Shutting down")
    # cv2.destroyAllWindows()
