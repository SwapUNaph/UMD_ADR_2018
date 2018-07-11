#!/usr/bin/env python

#  --- Changelog ---
# Goal:     Input by joystick. Startup drone by publishing to the state machine. Intervene by publishing to driving.
# Status:   06/19:  Manual flight possible through joystick. Takeoff with T1, Land with T2, Emergency land at Mode A, Move camera with hat.
#           06/25:  Start of script delayed until connected to state_machine. Manual flight possible through joystick. Takeoff with T1, Land with T2, Emergency land at Mode A, Manual flight with T3, automatic flight with T4.
#           07/10:  Not fully working without controller yet

import rospy
from std_msgs.msg import Empty
from geometry_msgs.msg import Twist
import signal
import sys
import time
from std_msgs.msg import Bool, Int32
import getch


def signal_handler(signal, frame):
    sys.exit(0)


def publish_status(st):
    pub = rospy.Publisher('/bebop/' + st, Empty, queue_size=1, latch=True)

    msg = Empty()
    pub.publish(msg)
    print('publish to /bebop/' + st)
    rospy.loginfo('publish to /bebop/' + st)


def callback_state_machine_changed(data):
    # used for initialization between jetson and ground
    global state_machine
    state_machine = data.data

    if state_machine == 0:
        time.sleep(1)
        # print("I heard from the drone")
        pub = rospy.Publisher('/auto/state_machine', Int32, queue_size=1, latch=True)
        # print('I will tell her')
        pub.publish(1)
    #elif state_machine == 2:
    #    pub = rospy.Publisher('/auto/state_machine', Int32, queue_size=1, latch=True)
    #    pub.publish(3)


def autonomy_pub(bool):
    pub = rospy.Publisher('/auto/autonomous_driving', Bool, queue_size=1, latch=True)

    pub.publish(bool)
    print('autonomy_active: ' + str(bool))
    rospy.loginfo('autonomy_active: ' + str(bool))


def process(key):
    if key == 'a':
        autonomy_pub(True)
        print("autonomy True")
    elif key == 'm':
        autonomy_pub(True)
        print("autonomy False")
    elif key == 'l':
        print("land")
        publish_status("land")
    elif key == 'q':
        exit('exitting')
    elif key == 'r':
        print("reset")
        publish_status("reset")
    print("Press\n a - autonomous\n m - manual\n l - landing\n r - reset\n q - quit")


def main():
    # Enable killing the script with Ctrl+C.
    signal.signal(signal.SIGINT, signal_handler)

    rospy.init_node('ground_output', anonymous=True)

    # Global variables for autonomy mode, and the status of the drone and the state machine
    global autonomy_active
    autonomy_active = False
    global state_machine
    state_machine = -1

    rospy.Subscriber("/auto/state_machine", Int32, callback_state_machine_changed)

    # Wait until connection between ground and air is established
    while state_machine <= 1:
        rospy.loginfo("waiting")
        time.sleep(0.5)

    print("GCS communicating")
    rospy.loginfo("GCS communicating")

    print("Press\n a - autonomous\n m - manual\n l - landing\n r - reset\n q - quit")
    while True:
        key = getch.getch()
        process(key)


if __name__ == '__main__':
    main()
