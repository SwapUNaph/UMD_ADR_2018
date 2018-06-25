#!/usr/bin/env python

#  --- Changelog ---
# Goal:     Input is start signal from ground_output and gate crossing confirmation. Output is a state on a global map, the current position, the upcoming gate type and similar
# Status:   06/19:  Not existing
#           06/25:  Start delayed until connected to ground_output. Observes takeoff and landing (and advances state)
#           06/25:

from __future__ import print_function
import sys
import rospy
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Empty, Bool, Int32
from bebop_msgs.msg import Ardrone3PilotingStateFlyingStateChanged
import time


# def callback_autonomous_driving(data):
#    print("autonomy_active: " + str(data.data))
#    rospy.loginfo("autonomy_active: " + str(data.data))


def callback_bebop_state_changed(data):
    # observe takeoff and landing
    global state_publisher

    if data.state == 2 and state_machine == 3:  # drone is hovering and was taking off
        print("takeoff completed")
        state_publisher.publish(4)

    if data.state == 0 and state_machine == 5:  # drone has landed and was landing
        print("landing completed")
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
    # used for initialization between jetson and ground
    global state_machine
    state_machine = data.data

    if state_machine == 0:
        pass
        # rospy.loginfo("I talked to ground")
        # print("I talked to ground")
    elif state_machine == 1:
        # rospy.loginfo("I heard back from ground")
        time.sleep(0.5)
        state_publisher.publish(2)
    elif state_machine == 2:
        pass


def main():
    rospy.init_node('state_machine', anonymous=True)

    # create global state publisher
    global state_publisher
    state_publisher = rospy.Publisher("/auto/state_machine", Int32, queue_size=3, latch=True)

    # rospy.Subscriber("/auto/autonomous_driving", Bool, callback_autonomous_driving)
    rospy.Subscriber("/bebop/states/ardrone3/PilotingState/FlyingStateChanged", Ardrone3PilotingStateFlyingStateChanged,
                     callback_bebop_state_changed)
    rospy.Subscriber("/auto/state_machine", Int32, callback_state_machine_changed)

    global state_machine
    state_machine = None

    # initializes startup by publishing state 0
    state_publisher.publish(0)

    # wait until communication with ground is established
    while state_machine <= 1:
        time.sleep(0.5)

    print("Jetson communicating")
    rospy.loginfo("Jetson communicating")

    # wait
    rospy.spin()

    print("Shutting down")
    # cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
