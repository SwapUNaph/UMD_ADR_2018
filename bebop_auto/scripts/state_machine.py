#!/usr/bin/env python

#  --- Changelog ---
# Goal:     Input is start signal from ground_output and gate crossing confirmation. Output is a state on a global map, the current position, the upcoming gate type and similar
# Status:   06/19:  Not existing
#           06/25:  Start delayed until connected to ground_output. Observes takeoff and landing (and advances state)
#           06/27:  State machine is now handling more/most state changes

import sys
import rospy
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Empty, Bool, Int32
from bebop_msgs.msg import Ardrone3PilotingStateFlyingStateChanged
import time
import signal


def signal_handler(signal, frame):
    sys.exit(0)


def callback_states_changed(data, args):
    global state_publisher
    global state_machine
    global bebop_state

    # update variables
    if args == "bebop_state":
        bebop_state = data.state
    elif args == "state_machine":
        state_machine = data.data

    # STATE MACHINE
    if state_machine == 0:                          # My own signal
        pass
    elif state_machine == 1:                        # Heard from ground -> respond
        time.sleep(1)
        state_publisher.publish(2)
    elif state_machine == 2 and bebop_state == 1:   # drone is taking off
        rospy.loginfo("takeoff started")
        state_publisher.publish(3)
    elif state_machine == 3 and bebop_state == 2:   # drone was taking off and is now hovering
        rospy.loginfo("takeoff completed")
        state_publisher.publish(4)
    elif state_machine == 4:                        # state transition in gate_crossing_detection
        pass
    elif state_machine == 5 and bebop_state == 4:   # drone has reached target and is now landing
        rospy.loginfo("landing started")
        state_publisher.publish(6)
    elif state_machine == 6 and bebop_state == 0:   # drone was landing and has landed
        rospy.loginfo("landing completed")
        state_publisher.publish(7)

    # STATE MACHINE overview
    #   0   state machine sent signal to ground
    #   1   ground received signal and sends it back to air
    #   2   air received response and starts autonomous takeoff
    #   3   drone is taking off
    #   4   takeoff completed, start mission
    #   5   mission finished, land
    #   6   landing
    #   7   landing completed

    # BEBOP STATE overview
    #   0   landed
    #   1   takeoff
    #   2   hovering
    #   3   flying
    #   4   landing
    #   5   emergency
    #   6   not observed (usertakeoff, User take off state. Waiting for user action to take off)
    #   7   not observed (for fixed wing, motor ramping state)
    #   8   not observed (emergency landinng after sensor defect. Only YAW is taken into account)


def main():
    # Enable killing the script with Ctrl+C.
    signal.signal(signal.SIGINT, signal_handler)

    rospy.init_node('state_machine', anonymous=True)

    # create global state publisher
    global state_publisher
    state_publisher = rospy.Publisher("/auto/state_machine", Int32, queue_size=3, latch=True)

    global state_machine
    global bebop_state
    state_machine = None
    bebop_state = None

    # rospy.Subscriber("/auto/autonomous_driving", Bool, callback_autonomous_driving)
    rospy.Subscriber("/bebop/states/ardrone3/PilotingState/FlyingStateChanged", Ardrone3PilotingStateFlyingStateChanged,
                     callback_states_changed, "bebop_state")
    rospy.Subscriber("/auto/state_machine", Int32, callback_states_changed, "state_machine")

    # initializes startup by publishing state 0
    state_publisher.publish(0)

    # wait until communication with ground is established
    while state_machine <= 1:
        time.sleep(0.5)

    rospy.loginfo("Jetson communicating")

    # wait
    rospy.spin()


if __name__ == '__main__':
    main()
