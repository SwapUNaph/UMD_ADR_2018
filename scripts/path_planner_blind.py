#!/usr/bin/env python

#  --- Changelog ---
# Goal:     Input current state and create a path using the information of the map and track layout. Used only if no gate was detected. Publishes path.
# Status:   06/19: Not existing
#           06/25: Takes in position and state
#           06/26: Creates a fake gate location at the very first time it is started and keeps publishing this location as blind path. Transform not working correctly


from geometry_msgs.msg import Pose, PoseArray, PoseStamped
from std_msgs.msg import Int32
# roslib.load_manifest('learning_tf')
import rospy
import time
import math
import numpy as np
import common_resources as cr


def position_updated(drone_pose):
    global fake_target_set

    if current_state <= 3:
        # publish zeros before mission start
        wp = Pose()
        wp.position.x = None
        publisher.publish(wp)

    if current_state == 4:
        if not fake_target_set:
            rospy.loginfo("set fake target")

            # implementation exactly reverse as in matlab. Invert necessary when not required in matlab vice versa
            drone_pos = [drone_pose.position.x, drone_pose.position.y, drone_pose.position.z]
            q = [drone_pose.orientation.x, drone_pose.orientation.y, drone_pose.orientation.z, drone_pose.orientation.w]
            qi = [-q[0], -q[1], -q[2], q[3]]

            # own heading:
            # dx0 = [1, 0, 0]
            # dy0 = [0, 1, 0]
            # dz0 = [0, 0, 1]
            # dx = qv_mult(q, dx0)
            # dy = qv_mult(q, dy0)
            # dz = qv_mult(q, dz0)

            gate_local = [-0.5, 0, 0.5]
            gate_global = cr.qv_mult(q, gate_local) + drone_pos
            gate_global = gate_global.tolist()
            rospy.loginfo("global position: " + str(gate_global))

            # initialize with this position
            global target
            target = gate_global
            fake_target_set = True

        wp = Pose()
        wp.position.x = target[0]
        wp.position.y = target[1]
        wp.position.z = target[2]

        publisher.publish(wp)


def state_updated(data):
    global current_state
    global target
    current_state = data.data


def main():
    rospy.init_node('path_planner_blind', anonymous=True)

    global current_state
    current_state = -1
    rospy.Subscriber("/auto/state_machine", Int32, state_updated)
    rospy.Subscriber("/auto/odometry_merged", Pose, position_updated)
    global publisher
    publisher = rospy.Publisher("/auto/path_blind", Pose, queue_size=2)

    global fake_target_set
    fake_target_set = False

    rospy.spin()


if __name__ == '__main__':
    main()
