#!/usr/bin/env python

#  --- Changelog ---
# Goal:     Input of both path planners. Find relevant waypoint depending on current position. Use visual path if possible, otherwise use blind backup. Calculate driving command based on waypoint position and orientation. Create driving commands
# Status:   06/19: Not existing
#           06/26: Reads updates from blind path, visual path and position. Publishes empty command

from geometry_msgs.msg import Pose, PoseArray, PoseStamped
from std_msgs.msg import Int32
# roslib.load_manifest('learning_tf')
import rospy
import tf
import math
import time
import signal
import sys
import numpy as np
from bebop_auto.msg import Auto_Driving_Msg


def signal_handler(signal, frame):
    sys.exit(0)



def qv_mult(q1, v1):
    length = math.sqrt(v1[0] * v1[0] + v1[1] * v1[1] + v1[2] * v1[2])
    v1 = tf.transformations.unit_vector(v1)
    q2 = list(v1)
    q2.append(0.0)
    return tf.transformations.quaternion_multiply(tf.transformations.quaternion_multiply(q1, q2),
                                                  tf.transformations.quaternion_conjugate(q1))[:3] * length


def received_update(data, args):
    #print(drone_pos.position.x, drone_pos.position.y, drone_pos.position.z, drone_pos.orientation.x,
    #      drone_pos.orientation.y, drone_pos.orientation.z, drone_pos.orientation.w)
    global drone
    global path_visual
    global path_blind

    if args == "position":
        # print("pos")
        drone = data
    elif args == "path_visual":
        # print("visual")
        path_visual = data
    elif args == "path_blind":
        # print("blind")
        path_blind = data


def callback_state_machine_changed(data):
    # used for initialization between jetson and ground
    global state_machine
    state_machine = data.data


def main():
    # Enable killing the script with Ctrl+C.
    signal.signal(signal.SIGINT, signal_handler)

    rospy.init_node('navigation', anonymous=True)

    global drone
    global path_visual
    global path_blind
    global state_machine
    drone = None
    path_visual = None
    path_blind = None
    state_machine = None
    rospy.Subscriber("/auto/odometry_merged", Pose, received_update, "position")
    rospy.Subscriber("/auto/path_blind", Pose, received_update, "path_blind")
    rospy.Subscriber("/auto/path_visual", Pose, received_update, "path_visual")
    rospy.Subscriber("/auto/state_machine", Int32, callback_state_machine_changed)

    driver_publisher = rospy.Publisher("/auto/auto_drive", Auto_Driving_Msg, queue_size=1)
    state_publisher = rospy.Publisher("/auto/state_machine", Int32, queue_size=3, latch=True)

    rate = rospy.Rate(20)

    while True:

        # set applicable path
        if path_visual is None:
            if path_blind is not None:
                path = path_blind
            else:
                print "no path"
                continue
        else:
            path = path_visual

        if drone is None:
            print "no position"
            continue

        # calculate path to WP
        diff_global = [path.position.x - drone.position.x,
                       path.position.y - drone.position.y,
                       path.position.z - drone.position.z]
        print diff_global

        distance = math.sqrt(diff_global[0] * diff_global[0] + diff_global[1] * diff_global[1] + diff_global[2] * diff_global[2])

        msg = Auto_Driving_Msg()

        if distance > 0.1:
            # implementation exactly reverse as in matlab. Invert necessary when not required in matlab vice versa
            q = [drone.orientation.x, drone.orientation.y, drone.orientation.z, drone.orientation.w]
            qi = [-q[0], -q[1], -q[2], q[3]]

            diff_bebop = qv_mult(qi, diff_global)
            # print("diff_bebop: " + str(diff_bebop))
            heading = math.atan2(-diff_bebop[1], diff_bebop[0]) * 180 / math.pi
            # print("heading to goal " + str(heading))

            # change driving message
            max = 0.5
            gain = 0.5/2

            msg.x = min(gain*diff_bebop[0], max)
            msg.y = min(gain*diff_bebop[1], max)
            msg.z = min(gain*diff_bebop[2], max)
            msg.r = 0

            print("Forward: " + str(msg.x))
            print("Left:    " + str(msg.y))
            print("Up:      " + str(msg.z))
            driver_publisher.publish(msg)

        else:
            # send empty driving message
            driver_publisher.publish(msg)
            # advance state machine
            if state_machine == 4:
                state_publisher.publish(5)

        rate.sleep()


if __name__ == '__main__':
    main()
