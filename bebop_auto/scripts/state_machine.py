#!/usr/bin/env python
from __future__ import print_function

import sys
import rospy
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Empty, Bool, Int32
from bebop_msgs.msg import Ardrone3PilotingStateFlyingStateChanged





def callback_autonomous_driving(data):


    print("autonomous driving")
    print(data)



def callback_flying_state_changed(data):


    print("flying state")
    print(data)

    # state_landed = 0  # Landed state
    # state_takingoff = 1  # Taking off state
    # state_hovering = 2  # Hovering / Circling (for fixed wings) state
    # state_flying = 3  # Flying state
    # state_landing = 4  # Landing state
    # state_emergency = 5  # Emergency state
    # state_usertakeoff = 6  # User take off state. Waiting for user action to take off.
    # state_motor_ramping = 7  # Motor ramping state (for fixed wings).
    # state_emergency_landing = 8  # Emergency landing state. Drone autopilot has detected defective sensor(s). Only Yaw argument in PCMD is taken into account. All others flying commands are ignored.





if __name__ == '__main__':

    rospy.init_node('state_machine', anonymous=True)
    publisher = rospy.Publisher("/auto/state", Int32, queue_size=1)

    rospy.Subscriber("/auto/autonomous_driving", Bool, callback_autonomous_driving)
    rospy.Subscriber("/states/ardrone3/PilotingState/FlyingStateChanged", Ardrone3PilotingStateFlyingStateChanged, callback_flying_state_changed)


    rospy.spin()

    print("Shutting down")
    # cv2.destroyAllWindows()
