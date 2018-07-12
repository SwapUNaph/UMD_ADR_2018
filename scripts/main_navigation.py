#!/usr/bin/env python

#  --- Changelog ---
# Goal:     Combine many other packages
# Status:   07/11: Started with script

from geometry_msgs.msg import Pose, PoseArray, PoseStamped
from std_msgs.msg import Int32, Float32MultiArray
from bebop_auto.msg import Waypoint_Msg
# roslib.load_manifest('learning_tf')
import rospy
import math
import time
from bebop_msgs.msg import Ardrone3PilotingStateFlyingStateChanged
import signal
import sys
import numpy as np
from tf import transformations as tfs
from bebop_auto.msg import Auto_Driving_Msg

import common_resources as cr

def signal_handler(signal, frame):
    sys.exit(0)


def callback_states_changed(data, args):
    # update variables
    if args == "state_auto":
        global state_auto
        state_auto = data.data
    elif args == "state_bebop":
        global state_bebop
        state_bebop = data.state


def callback_odometry_merged_changed(data):
    global odometry_merged
    odometry_merged = data


def calculate_blind_wp(wp_blind):
    if state_auto == 2:
        if wp_blind is None:
            rospy.loginfo("set fake target")

            # implementation exactly reverse as in matlab. Invert necessary when not required in matlab vice versa
            drone_pos = [odometry_merged.position.x, odometry_merged.position.y, odometry_merged.position.z]
            q = [odometry_merged.orientation.x, odometry_merged.orientation.y, odometry_merged.orientation.z,
                 odometry_merged.orientation.w]
            qi = [-q[0], -q[1], -q[2], q[3]]

            gate_local = [0.5, 0, 1]
            gate_global = cr.qv_mult(q, gate_local) + drone_pos
            gate_global = gate_global.tolist()
            rospy.loginfo("global position: " + str(gate_global))

            # initialize with this position
            wp_blind = cr.WP(gate_global, 0)
    return wp_blind


def select_waypoint(wp_visual, wp_blind):
    if wp_visual is not None:
        return wp_visual
    else:
        return wp_blind

def navigate(odometry_merged, wp):
    # calculate path to WP
    diff_global = [wp.x - odometry_merged.position.x,
                   wp.y - odometry_merged.position.y,
                   wp.z - odometry_merged.position.z]

    distance = math.sqrt(diff_global[0]*diff_global[0] + diff_global[1]*diff_global[1] + diff_global[2]*diff_global[2])

    # implementation exactly reverse as in matlab. Invert necessary when not required in matlab vice versa
    q = [odometry_merged.orientation.x, odometry_merged.orientation.y, odometry_merged.orientation.z, odometry_merged.orientation.w]
    qi = [-q[0], -q[1], -q[2], q[3]]

    diff_bebop = cr.qv_mult(qi, diff_global)
    # print("heading to goal " + str(math.atan2(-diff_bebop[1], diff_bebop[0]) * 180 / math.pi))

    # change driving message
    limit = 0.1
    gain = 0.1 / 0.5

    auto_driving_msg = Auto_Driving_Msg()
    auto_driving_msg.x = min(gain * diff_bebop[0], limit)
    auto_driving_msg.y = min(gain * diff_bebop[1], limit)
    auto_driving_msg.z = min(gain * diff_bebop[2], limit)
    auto_driving_msg.r = 0

    return auto_driving_msg, distance


if __name__ == '__main__':
    # Enable killing the script with Ctrl+C.
    signal.signal(signal.SIGINT, signal_handler)

    rospy.init_node('main_navigation', anonymous=True)
    rate = rospy.Rate(20)

    # Publishers
    state_auto_publisher      = rospy.Publisher("/auto/state_auto",      Int32,            queue_size=1, latch=True)
    odometry_merged_publisher = rospy.Publisher("/auto/odometry_merged", Pose,             queue_size=1, latch=True)
    auto_drive_publisher      = rospy.Publisher("/auto/auto_drive",      Auto_Driving_Msg, queue_size=1, latch=True)

    # Subscribers
    rospy.Subscriber("/bebop/states/ardrone3/PilotingState/FlyingStateChanged", Ardrone3PilotingStateFlyingStateChanged, callback_states_changed, "state_bebop")
    rospy.Subscriber("/auto/state_auto", Int32, callback_states_changed, "state_auto")
    rospy.Subscriber("/auto/odometry_merged", Pose, callback_odometry_merged_changed)

    # Variables
    state_auto = None
    state_bebop = None
    odometry_merged = None
    wp_visual = None
    wp_blind = None
    navigation_active = False

    # initializes startup by publishing state 0
    state_auto_publisher.publish(0)

    # Wait until connection between ground and air is established
    while state_auto is None:
        state_auto_publisher.publish(0)
        rospy.loginfo("waiting None")
        time.sleep(0.5)
    while state_auto == 0:
        rospy.loginfo("waiting 0")
        time.sleep(0.5)
    while state_auto == 1:
        state_auto_publisher.publish(state_auto + 1)
        rospy.loginfo("waiting 1")
        time.sleep(0.5)

    rospy.loginfo("Jetson communicating")

    while True:
        if navigation_active:
            # send real commands
            # print("fly: " + str(auto_driving_msg))
            auto_drive_publisher.publish(auto_driving_msg)
        else:
            # send empty commands
            # print("fly: disabled")
            auto_drive_publisher.publish(Auto_Driving_Msg())

        rate.sleep()

        # ensure there is an odometry_merged
        if odometry_merged is None:
            rospy.loginfo("No odometry")
            auto_drive_publisher.publish(Auto_Driving_Msg())
            continue

        # calculate visual wp

        # calculate blind wp
        wp_blind = calculate_blind_wp(wp_blind)

        # ensure there is a waypoint
        if wp_visual is None and wp_blind is None:
            rospy.loginfo("No waypoints")
            auto_drive_publisher.publish(Auto_Driving_Msg())
            continue

        # select applicable waypoint
        wp = select_waypoint(wp_visual, wp_blind)

        # navigate to wp
        [auto_driving_msg, distance] = navigate(odometry_merged, wp)

        # state_machine_advancement (if conditions are met: distances, states, ...)

        if state_auto == 2 and state_bebop == 1:    # drone is taking off
            rospy.loginfo("takeoff started")
            state_auto_publisher.publish(state_auto + 1)
        elif state_auto == 3 and state_bebop == 2:  # drone was taking off and is now hovering/flying
            navigation_active = True
            rospy.loginfo("takeoff completed")
            state_auto_publisher.publish(state_auto + 1)
        elif state_auto == 4 and distance < 0.1:       # drone reached waypoint
            wp_blind = None
            navigation_active = False
            rospy.loginfo("mission finished")
            state_auto_publisher.publish(state_auto + 1)
        elif state_auto == 5 and state_bebop == 4:  # drone has reached target and is now landing
            rospy.loginfo("landing started")
            state_auto_publisher.publish(state_auto + 1)
        elif state_auto == 6 and state_bebop == 0:  # drone was landing and has landed
            rospy.loginfo("landing completed")
            state_auto_publisher.publish(state_auto + 1)

##### old state machine!
    # STATE MACHINE overview
    #   0   state machine sent signal to ground
    #   1   ground received signal and sends it back to air
    #   2   air received response and starts autonomous takeoff
    #   3   drone is taking off
    #   4   takeoff completed, start mission blind (0.5,0.0,0.5). after 15s, gate will be detected 1.5m away at 45deg
    #   5   too close to WP (.5m), so continuing towards WP until reached
    #   6   WP reached (.05m), continue in gate direction (north) for 1m
    #   7   location reached, mission finished, land
    #   8   landing
    #   9   landing completed

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



## from gate crossing detection

        # if state_machine == 5:
        #     if distance < 0.5:
        #         rospy.loginfo("too close to WP")
        #         print(path)
        #         state_publisher.publish(state_machine + 1)
        #
        # elif state_machine == 6:
        #     if distance < 0.1:
        #         rospy.loginfo("Target reached")
        #         state_publisher.publish(state_machine + 1)
        #
        # elif state_machine == 7:
        #     if distance < 0.1:
        #         rospy.loginfo("landing reached")
        #         state_publisher.publish(state_machine + 1)







# waypoint craetion blind:
#
#     rospy.init_node('waypoint_creation_blind', anonymous=True)
#
#     global current_state
#     global wp_visual
#     current_state = -1
#     wp_visual = None
#     rospy.Subscriber("/auto/state_machine", Int32, state_updated)
#     rospy.Subscriber("/auto/odometry_merged", Pose, position_updated)
#     rospy.Subscriber("/auto/wp_visual", Waypoint_Msg, wp_visual_updated)
#     global publisher
#     publisher = rospy.Publisher("/auto/wp_blind", Waypoint_Msg, queue_size=1)
#
#     global fake_target_set
#     fake_target_set = False
#
# def position_updated(drone_pose):
#     global fake_target_set
#     global wp_visual
#
#     msg = Waypoint_Msg()
#
#     if current_state <= 3 or current_state == 5:
#         # publish zeros before mission start
#         publisher.publish(msg)
#
#     if current_state == 4:
#         if not fake_target_set:
#             rospy.loginfo("set fake target")
#
#             # implementation exactly reverse as in matlab. Invert necessary when not required in matlab vice versa
#             drone_pos = [drone_pose.position.x, drone_pose.position.y, drone_pose.position.z]
#             q = [drone_pose.orientation.x, drone_pose.orientation.y, drone_pose.orientation.z, drone_pose.orientation.w]
#             qi = [-q[0], -q[1], -q[2], q[3]]
#
#             gate_local = [0.5, 0, 0]
#             gate_global = cr.qv_mult(q, gate_local) + drone_pos
#             gate_global = gate_global.tolist()
#             rospy.loginfo("global position: " + str(gate_global))
#
#             # initialize with this position
#             global target
#             target = gate_global
#             fake_target_set = True
#
#         msg.pos = target
#         msg.hdg = 0
#         publisher.publish(msg)
#
#     if current_state == 6:#or current_state == 5:
#         # continue towards gate location blindly
#         print("continue blindly")
#         print(msg)
#         if wp_visual.pos is not ():
#             msg = wp_visual
#             publisher.publish(msg)
#
#     if current_state == 7:
#         # continue in gate direction for 0.5m
#         old_wp_pos = wp_visual[:3]
#         old_wp_hdg = wp_visual[3]
#
#         extra_distance = np.array([0.5*math.cos(old_wp_hdg), 0.5*math.sin(old_wp_hdg), 0])
#
#         new_wp_pos = extra_distance + old_wp_pos
#         new_wp_hdg = old_wp_hdg
#
#         msg.pos = new_wp_pos
#         msg.hdg = new_wp_hdg
#         publisher.publish(msg)
#         rospy.loginfo("land at: " + str(msg.pos))
#
#
#



