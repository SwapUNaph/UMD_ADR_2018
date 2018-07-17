#!/usr/bin/env python

#  --- Changelog ---
# Goal:     Combine many other packages
# Status:   07/11: Started with script

import rospy
import signal
import sys
import math
import numpy as np
import time
from std_msgs.msg import Int32
from geometry_msgs.msg import Pose
from bebop_msgs.msg import Ardrone3PilotingStateFlyingStateChanged
from bebop_auto.msg import Auto_Driving_Msg, Gate_Detection_Msg
from tf import transformations as tfs
import common_resources as cr


def signal_handler(signal, frame):
    sys.exit(0)


def callback_states_changed(data, args):
    # update variables
    if args == "state_auto":
        global state_auto
        state_auto = data.data
        rospy.loginfo("state auto changed to " + str(state_auto))
    elif args == "state_bebop":
        global state_bebop
        state_bebop = data.state
        rospy.loginfo("state bebop changed to " + str(state_bebop))


def callback_visual_detection_changed(data):
    global gate_detection_info
    gate_detection_info = cr.Gate_Detection_Info(data)


def callback_odometry_merged_changed(data):
    global odometry_merged
    odometry_merged = data


def calculate_visual_wp(wp_visual, wp_visual_old, gate_detection_info, wp_visual_history):
    wp_visual_temp = wp_visual


    # from tf import transformations as tfs


    # maybe average over latest gates
    # global latest_gates
    # latest_gates = [[0,0,0,0]]

    #
    rospy.loginfo('gate_detection_info')
    rospy.loginfo(gate_detection_info)
    if gate_detection_info is None or gate_detection_info.rvec == ():
        wp_visual = None
    else:
        # bebop position and orientation
        bebop_pos = [[gate_detection_info.bebop_pose.position.x], [gate_detection_info.bebop_pose.position.y], [gate_detection_info.bebop_pose.position.z]]
        bebop_q = [gate_detection_info.bebop_pose.orientation.x, gate_detection_info.bebop_pose.orientation.y,
                   gate_detection_info.bebop_pose.orientation.z, gate_detection_info.bebop_pose.orientation.w]
        # bebop_x_vec = cr.qv_mult(bebop_q, [1, 0, 0])
        # print 'own_heading'
        # print math.atan2(-bebop_x_vec[1], bebop_x_vec[0])*180/math.pi

        # gate position and orientation
        gate_pos = gate_detection_info.tvec
        gate_q = cr.axang2quat(gate_detection_info.rvec)

        gate_global_p = cr.qv_mult(bebop_q, cr.qv_mult(cr.cam_q, gate_pos) + cr.BZ) + bebop_pos
        gate_global_q = tfs.quaternion_multiply(bebop_q, tfs.quaternion_multiply(cr.cam_q, gate_q))
        gate_normal_vec = cr.qv_mult(gate_global_q, [0, 0, 1])
        heading_to_gate = math.atan2(-(gate_global_p[1]-bebop_pos[1]),gate_global_p[0]-bebop_pos[0])
        heading_of_gate = math.atan2(-gate_normal_vec[1],gate_normal_vec[0])
        heading_difference = math.fabs(heading_to_gate - heading_of_gate)*180/math.pi

        if 90 < heading_difference < 270:
            if heading_of_gate < 0:
                heading_of_gate = heading_of_gate + math.pi
            else:
                heading_of_gate = heading_of_gate - math.pi

        current_wp = cr.WP(gate_global_p, heading_of_gate)

        distance = 999
        if wp_visual_history is not None:
            distance = cr.length(gate_global_p - wp_visual_history[-1].pos)

        if distance > 0.5:
            wp_visual_history = [current_wp]
        else:
            wp_visual_history.append(current_wp)
            if len(wp_visual_history) > 10:
                del wp_visual_history[0]

        wp_visual = cr.find_average(wp_visual_history)

    rospy.loginfo("wp_visual")
    rospy.loginfo(wp_visual)
    return wp_visual, wp_visual_temp, wp_visual_history


def calculate_blind_wp(wp_blind, wp_blind_old, wp_visual, wp_visual_old):
    wp_blind_temp = wp_blind
    if state_auto == 4:
        rospy.loginfo("state 4")
        if wp_blind is None:
            rospy.loginfo("set fake target")

            # implementation exactly reverse as in matlab. Invert necessary when not required in matlab vice versa
            bebop_pos = [odometry_merged.position.x, odometry_merged.position.y, odometry_merged.position.z]
            bebop_q = [odometry_merged.orientation.x, odometry_merged.orientation.y, odometry_merged.orientation.z,
                 odometry_merged.orientation.w]

            dx0 = [1, 0, 0]
            dx = cr.qv_mult(bebop_q, dx0)
            old_heading = math.atan2(-dx[1], dx(0))
            new_heading = old_heading + math.pi/2
            if new_heading > 2*math.pi:
                new_heading = new_heading - 2*math.pi

            blind_position = [0.5, 0, 0]
            blind_position_global = cr.qv_mult(bebop_q, blind_position) + bebop_pos
            blind_position_global = blind_position_global.tolist()

            # initialize with this position
            wp_blind = cr.WP(blind_position_global, new_heading)
            rospy.loginfo("global position set for blind")

    if state_auto == 5 and wp_visual is None:
        if wp_visual_old is not None:
            rospy.loginfo("state 5, set visual as blind")
            wp_blind = wp_visual_old

    if state_auto == 6 and wp_blind is None:
        rospy.loginfo("state 6, set new blind")
        # continue in gate direction for 0.5m

        extra_distance = 0.5 * np.array([math.cos(wp_blind_old.hdg), math.sin(wp_blind_old.hdg), 0])
        wp_blind = cr.WP(wp_blind_old.pos + extra_distance, wp_blind_old.hdg)

        rospy.loginfo("land at: " + str(wp_blind))

    rospy.loginfo("wp_blind")
    rospy.loginfo(wp_blind)
    return wp_blind, wp_blind_temp


def select_waypoint(wp_visual, wp_blind):
    if wp_visual is not None:
        rospy.loginfo("fly visual")
        return wp_visual
    elif wp_blind is not None:
        rospy.loginfo("fly blind")
        return wp_blind
    else:
        rospy.loginfo("no wp")
        return None


def navigate(odometry_merged, wp):

    # rospy.loginfo("fly from")
    # rospy.loginfo(odometry_merged)
    # rospy.loginfo("fly to")
    # rospy.loginfo(wp)
    # calculate path to WP
    # diff_global = wp.pos - [odometry_merged.pose.pose.position.x, odometry_merged.pose.pose.position.y, odometry_merged.pose.pose.position.z]

    # navigation_distance = cr.length(diff_global)
    # rospy.loginfo("distance " + str(navigation_distance))

    # implementation exactly reverse as in matlab. Invert necessary when not required in matlab vice versa
    # q = [odometry_merged.orientation.x, odometry_merged.orientation.y, odometry_merged.orientation.z, odometry_merged.orientation.w]
    # qi = [-q[0], -q[1], -q[2], q[3]]

    # diff_bebop = cr.qv_mult(qi, diff_global)
    # rospy.loginfo("heading to goal " + str(math.atan2(-diff_bebop[1], diff_bebop[0]) * 180 / math.pi))


    X_limit = .25
    Y_limit = .4
    Z_limit = .75
    R_limit = 1

    quat = odometry_merged.pose.pose.orientation
    angle = tf.transformations.euler_from_quaternion([quat.x,quat.y,quat.z,quat.w])[2]

    global_vel = odometry_merged.twist.twist.linear
    velocity = [global_vel.x*math.cos(angle) - global_vel.y*math.sin(angle),
                -global_vel.y*math.cos(angle) - global_vel.x*math.sin(angle),
                global_vel.z]
    # velocity = odometry_merged.twist.twist.linear
    

    diff_global = [wp.pos[0] - odometry_merged.pose.pose.position.x, wp.pos[1] - odometry_merged.pose.pose.position.y, wp.pos[3] - odometry_merged.position.x,path.position.z]
    
    dist = math.hypot(diff_global[0],diff_global[1])
    
    quat = odometry_merged.pose.pose.orientation
    angle = tf.transformations.euler_from_quaternion([quat.x,quat.y,quat.z,quat.w])[2]

    gate_theta = wp.hdg

    pos_theta = math.atan2(diff_global[1],diff_global[0])

    d_theta = gate_theta-pos_theta
    if d_theta > 180:
        d_theta = 360-d_theta
    elif d_theta < 180:
        d_theta = -360+d_theta
    else:
        pass


    y_vel_des = -dist*math.sin(d_theta)
    x_vel_des = -dist*math.cos(d_theta)
    z_error = -diff_global[2]

    
    r_error = pos_theta - euler[3]
    if r_error > 180:
        r_error = 360-r_error
    elif r_error < 180:
        r_error = -360+r_error


    y_vel_error = y_vel_des-velocity[1]
    x_vel_error = x_vel_des-velocity[0]
    
    msg = Auto_Driving_Msg()

    Xcmd = X_PID.update(x_vel_error)
    Ycmd = Y_PID.update(y_vel_error)

    # print ' P: ',Xcmd[0],' I: ', Xcmd[1],' D: ', Xcmd[2]
    # print ' P: ',Ycmd[0],' I: ', Ycmd[1],' D: ', Ycmd[2]
    
    msg.x = max(min(Xcmd[0]+Xcmd[1]+Xcmd[2], X_limit),-X_limit)
    msg.y = max(min(Ycmd[0]+Ycmd[1]+Ycmd[2], Y_limit),-Y_limit)
    msg.z = max(min(Z_PID.update(z_error), Z_limit),-Z_limit)
    msg.r = max(min(R_PID.update(t_error), R_limit),-R_limit)
    

    driver_publisher.publish(msg)
    # rospy.loginfo("calculated")
    # rospy.loginfo(auto_driving_msg)
    return auto_driving_msg


def limit_value(value, limit):
    if value > limit:
        return limit
    elif value < -limit:
        return limit
    else:
        return value


def state_machine_advancement(state_auto, state_bebop, navigation_distance, state_auto_publisher, navigation_active, wp_blind, wp_visual):
    # BEBOP STATE overview
    #   0   landed
    #   1   takeoff
    #   2   hovering
    #   3   flying
    #   4   landing
    #   5   emergency
    #   6   not observed (usertakeoff, User take off state. Waiting for user action to take off)
    #   7   not observed (for fixed wing, motor ramping state)
    #   8   not observed (emergency landing after sensor defect. Only YAW is taken into account)

    # STATE MACHINE overview
    #   2   air received response from ground and starts autonomous takeoff
    #   3   drone is taking off
    #   4   takeoff completed, start mission blind (fly forward 0.5m and rotate +90 deg)
    #   5   gate has been detected on the right
    #   6   gate reached, continue in gate direction for 0.75m
    #   7   location reached, mission finished, land
    #   8   landing
    #   9   landing completed

    rospy.loginfo("state machine sees: auto " + str(state_auto) + " and bebop " + str(state_bebop))
    if state_auto == 2 and state_bebop == 1:        # drone is taking off
        rospy.loginfo("takeoff started")
        state_auto_publisher.publish(state_auto + 1)
    elif state_auto == 3 and state_bebop == 2:      # drone was taking off and is now hovering/flying
        navigation_active = True
        rospy.loginfo("takeoff completed")
        state_auto_publisher.publish(state_auto + 1)
    elif state_auto == 4 and wp_visual is not None: # we detected the gate and try to get there
        wp_blind = None
        rospy.loginfo("visually detected gate")
        state_auto_publisher.publish(state_auto + 1)
    elif state_auto == 5 and navigation_distance < 0.2:        # drone reached gate, continue in gate direction
        wp_blind = None
        rospy.loginfo("gate reached")
        state_auto_publisher.publish(state_auto + 1)
    elif state_auto == 6 and navigation_distance < 0.2:        # drone reached landing location
        wp_blind = None
        navigation_active = False
        rospy.loginfo("mission finished")
        state_auto_publisher.publish(state_auto + 1)
    elif state_auto == 7 and state_bebop == 4:      # drone initiated landing
        rospy.loginfo("landing started")
        state_auto_publisher.publish(state_auto + 1)
    elif state_auto == 8 and state_bebop == 0:      # drone was landing and has landed
        rospy.loginfo("landing completed")
        state_auto_publisher.publish(state_auto + 1)

    return [navigation_active, wp_blind]


if __name__ == '__main__':
    # Enable killing the script with Ctrl+C.
    signal.signal(signal.SIGINT, signal_handler)

    rospy.init_node('main_navigation', anonymous=True)
    rate = rospy.Rate(cr.frequency)

    # Variables
    state_auto = None
    state_bebop = None
    odometry_merged = None
    gate_detection_info = None
    wp_visual = None
    wp_visual_old = None
    wp_visual_history = None
    wp_blind = None
    wp_blind_old = None
    wp = None
    navigation_active = False
    empty_command = True
    auto_driving_msg = Auto_Driving_Msg()
    global X_PID
    global Y_PID
    global Z_PID
    global R_PID 
    X_PID = cr.PID(.08,0,0)
    Y_PID = cr.PID(.1,0,.5)
    Z_PID = cr.PID(1,0,0)
    R_PID = cr.PID(.5,0,1)


    # Publishers
    state_auto_publisher      = rospy.Publisher("/auto/state_auto",      Int32,            queue_size=1, latch=True)
    auto_drive_publisher      = rospy.Publisher("/auto/auto_drive",      Auto_Driving_Msg, queue_size=1, latch=True)

    # Subscribers
    rospy.Subscriber("/bebop/states/ardrone3/PilotingState/FlyingStateChanged", Ardrone3PilotingStateFlyingStateChanged, callback_states_changed, "state_bebop")
    rospy.Subscriber("/auto/state_auto", Int32, callback_states_changed, "state_auto")
    rospy.Subscriber("/auto/odometry_merged", Pose, callback_odometry_merged_changed)
    rospy.Subscriber("/auto/gate_detection_result", Gate_Detection_Msg, callback_visual_detection_changed)

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
    state_auto_publisher.publish(2)

    rospy.loginfo("Jetson communicating")

    while True:
        # These commands and the sleep are moved from the end (where you might expect),
        # so they will always be executed (also when script results in "continue")

        if empty_command:
            # send empty commands
            auto_drive_publisher.publish(Auto_Driving_Msg())
            rospy.loginfo("publish empty driving msg")
        else:
            # send real commands
            auto_drive_publisher.publish(auto_driving_msg)
            rospy.loginfo("publish real driving msg")
            rospy.loginfo(auto_driving_msg.x)
            rospy.loginfo(auto_driving_msg.y)
            rospy.loginfo(auto_driving_msg.z)
            rospy.loginfo(auto_driving_msg.r)

        rate.sleep()

        # state_machine_advancement (if conditions are met: distances, states, ...)
        if wp is not None and odometry_merged is not None:
            diff_global = wp.pos - [[odometry_merged.position.x], [odometry_merged.position.y], [odometry_merged.position.z]]
            navigation_distance = cr.length(diff_global)
        else:
            navigation_distance = 999
        rospy.loginfo("navigation_distance")
        rospy.loginfo(navigation_distance)
        [navigation_active, wp_blind] = state_machine_advancement(state_auto, state_bebop, navigation_distance, state_auto_publisher, navigation_active, wp_blind, wp_visual)


        # calculate visual wp
        rospy.loginfo("calculate visual WP")
        wp_visual, wp_visual_old, wp_visual_history = calculate_visual_wp(wp_visual, wp_visual_old, gate_detection_info, wp_visual_history)

        # calculate blind wp
        rospy.loginfo("calculate blind WP")
        wp_blind, wp_blind_old = calculate_blind_wp(wp_blind, wp_blind_old, wp_visual, wp_visual_old)

        # select applicable waypoint
        wp = select_waypoint(wp_visual, wp_blind)

        # ensure there is a waypoint
        if wp is None:
            rospy.loginfo("No waypoints")
            empty_command = True
            continue

        if not navigation_active:
            rospy.loginfo("Navigation turned off")
            empty_command = True
            continue

        empty_command = False

        # navigate to wp
        auto_driving_msg = navigate(odometry_merged, wp)

        # log = ""
        # log = log + str(state_auto) + "\t" + str(state_bebop) + "\t"
        #
        # if odometry_merged is None:
        #     log = log + None + "\t" + None + "\t" + None + "\t"
        # else:
        #     log = log + str(odometry_merged.position.x) + "\t" + str(odometry_merged.position.y) + "\t" + str(odometry_merged.position.z) + "\t"
        #
        # if wp_visual is None:
        #     log = log + None + "\t" + None + "\t" + None + "\t" + None + "\t"
        # else:
        #     log = log + str(wp_visual.x) + "\t" + str(wp_visual.y) + "\t" + str(wp_visual.z) + "\t" + str(wp_visual.hdg) + "\t"
        #
        # if wp_blind is None:
        #     log = log + None + "\t" + None + "\t" + None + "\t" + None + "\t"
        # else:
        #     log = log + str(wp_blind.x) + "\t" + str(wp_blind.y) + "\t" + str(wp_blind.z) + "\t" + str(wp_blind.hdg) + "\t"
        #
        # log = log + str(distance) + "\t"
        #
        # if navigation_active:
        #     log = log + str(auto_driving_msg.x) + "\t" + str(auto_driving_msg.y) + "\t" + str(auto_driving_msg.z) + "\t" + str(auto_driving_msg.r) + "\t"
        # else:
        #     log = log + None + "\t" + None + "\t" + None + "\t" + None + "\t"
        #
        # file_handler.write(log)




