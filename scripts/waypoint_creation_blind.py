#!/usr/bin/env python

#  --- Changelog ---
# Goal:     Input current state and create a path using the information of the map and track layout. Used only if no gate was detected. Publishes path.
# Status:   06/19: Not existing
#           06/25: Takes in position and state
#           06/26: Creates a fake gate location at the very first time it is started and keeps publishing this location as blind path. Transform not working correctly


from geometry_msgs.msg import Pose, PoseArray, PoseStamped
from std_msgs.msg import Int32, Float32MultiArray, MultiArrayDimension
from bebop_auto.msg import Waypoint_Msg
# roslib.load_manifest('learning_tf')
import rospy
import time
import math
import numpy as np
import common_resources as cr


def position_updated(drone_pose):
    global fake_target_set
    global wp_visual

    msg = Waypoint_Msg()

    if current_state <= 3:
        # publish zeros before mission start
        publisher.publish(msg)

    if current_state == 4:
        if not fake_target_set:
            rospy.loginfo("set fake target")

            # implementation exactly reverse as in matlab. Invert necessary when not required in matlab vice versa
            drone_pos = [drone_pose.position.x, drone_pose.position.y, drone_pose.position.z]
            q = [drone_pose.orientation.x, drone_pose.orientation.y, drone_pose.orientation.z, drone_pose.orientation.w]
            qi = [-q[0], -q[1], -q[2], q[3]]

            gate_local = [0.5, 0, 0.5]
            gate_global = cr.qv_mult(q, gate_local) + drone_pos
            gate_global = gate_global.tolist()
            rospy.loginfo("global position: " + str(gate_global))

            # initialize with this position
            global target
            target = gate_global
            fake_target_set = True

        msg.pos = target
        msg.hdg = 0
        publisher.publish(msg)

    if current_state == 5:
        # continue towards gate location blindly
        msg = wp_visual
        publisher.publish(msg)

    if current_state == 6:
        # continue in gate direction for 1m
        old_wp_pos = wp_visual[:3]
        old_wp_hdg = wp_visual[3]

        extra_distance = np.array([math.cos(old_wp_hdg), math.sin(old_wp_hdg), 0])

        new_wp_pos = extra_distance + old_wp_pos
        new_wp_hdg = old_wp_hdg

        msg.pos = new_wp_pos
        msg.hdg = new_wp_hdg
        publisher.publish(msg)


def state_updated(data):
    global current_state
    current_state = data.data


def wp_visual_updated(data):
    global wp_visual
    wp_visual = data


def main():
    rospy.init_node('waypoint_creation_blind', anonymous=True)

    global current_state
    global wp_visual
    current_state = -1
    wp_visual = None
    rospy.Subscriber("/auto/state_machine", Int32, state_updated)
    rospy.Subscriber("/auto/odometry_merged", Pose, position_updated)
    rospy.Subscriber("/auto/wp_visual", Waypoint_Msg, wp_visual_updated)
    global publisher
    publisher = rospy.Publisher("/auto/wp_blind", Waypoint_Msg, queue_size=1)

    global fake_target_set
    fake_target_set = False

    rospy.spin()


if __name__ == '__main__':
    main()
