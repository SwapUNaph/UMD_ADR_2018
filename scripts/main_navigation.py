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
from std_msgs.msg import Int32, String
from bebop_msgs.msg import Ardrone3PilotingStateFlyingStateChanged
from bebop_auto.msg import Auto_Driving_Msg, Gate_Detection_Msg, WP_Msg
from nav_msgs.msg import Odometry
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

    global auto_driving_msg
    global wp
    global min_distance
    global wp_visual
    global wp_visual_old
    global wp_visual_history
    global wp_blind
    global wp_blind_old
    global wp_look
    global navigation_active


    # state_machine_advancement (if conditions are met: distances, states, ...)
    if wp is not None and odometry_merged is not None:
        diff_global = wp.pos - [odometry_merged.pose.pose.position.x, odometry_merged.pose.pose.position.y, odometry_merged.pose.pose.position.z]
        navigation_distance = cr.length(diff_global)
    else:
        navigation_distance = 999
    rospy.loginfo("navigation distance")
    rospy.loginfo(navigation_distance)
    if navigation_distance < min_distance:
        min_distance = navigation_distance
    rospy.loginfo("min distance")
    rospy.loginfo(min_distance)
    [navigation_active, wp_blind, wp_loop] = state_machine_advancement(state_auto, state_bebop, navigation_distance, state_auto_publisher, navigation_active, wp_blind, wp_visual, wp_look)

    if odometry_merged is None:
        rospy.loginfo("No position")
        rospy.loginfo("publish empty driving msg")
        auto_drive_publisher.publish(Auto_Driving_Msg())
        return

    # calculate visual wp
    rospy.loginfo("calculate visual WP")
    wp_visual, wp_visual_old, wp_visual_history = calculate_visual_wp(wp_visual, wp_visual_old, gate_detection_info, wp_visual_history)

    # calculate blind wp
    rospy.loginfo("calculate blind WP")
    wp_blind, wp_blind_old, wp_look = calculate_blind_wp(wp_blind, wp_blind_old, wp_visual, wp_visual_old, wp_look)

    # select applicable waypoint
    wp = select_waypoint(wp_visual, wp_blind)

    # ensure there is a waypoint
    if wp is None:
        rospy.loginfo("No waypoints")
        rospy.loginfo("publish empty driving msg")
        auto_drive_publisher.publish(Auto_Driving_Msg())
        return

    # navigate to wp
    if navigation_active == "off":
        rospy.loginfo("Navigation turned off")
        rospy.loginfo("publish empty driving msg")
        auto_drive_publisher.publish(Auto_Driving_Msg())
        return
    elif navigation_active == "point":
        auto_driving_msg = navigate_point(odometry_merged, wp, wp_look)
    elif navigation_active == "throu":
        auto_driving_msg = navigate_throu(odometry_merged, wp)

    auto_drive_publisher.publish(auto_driving_msg)
    rospy.loginfo("publish real driving msg")
    rospy.loginfo([auto_driving_msg.x, auto_driving_msg.y, auto_driving_msg.z, auto_driving_msg.r])


def calculate_visual_wp(wp_visual, wp_visual_old, gate_detection_info, wp_visual_history):
    wp_visual_temp = wp_visual

    if gate_detection_info is None or (gate_detection_info.bebop_pose.position.x == 0 and gate_detection_info.bebop_pose.position.y == 0 and gate_detection_info.bebop_pose.position.z == 0):
        wp_visual = None
    else:
        # bebop position and orientation
        bebop_pos = [[gate_detection_info.bebop_pose.position.x], [gate_detection_info.bebop_pose.position.y], [gate_detection_info.bebop_pose.position.z]]
        bebop_q = [gate_detection_info.bebop_pose.orientation.x, gate_detection_info.bebop_pose.orientation.y,
                   gate_detection_info.bebop_pose.orientation.z, gate_detection_info.bebop_pose.orientation.w]

        # gate position and orientation

        rospy.loginfo("tvec")
        rospy.loginfo(gate_detection_info.tvec)
        rospy.loginfo("rvec")
        rospy.loginfo(gate_detection_info.rvec)

        gate_pos = gate_detection_info.tvec
        gate_q = cr.axang2quat(gate_detection_info.rvec)

        rospy.loginfo("bebop_pos")
        rospy.loginfo(bebop_pos)
        rospy.loginfo("bebop_q")
        rospy.loginfo(bebop_q)
        rospy.loginfo("gate_pos")
        rospy.loginfo(gate_pos)
        rospy.loginfo("gate_q")
        rospy.loginfo(gate_q)

        gate_global_p = cr.qv_mult(bebop_q, cr.qv_mult(cr.cam_q, gate_pos) + cr.BZ) + bebop_pos
        gate_global_q = tfs.quaternion_multiply(bebop_q, tfs.quaternion_multiply(cr.cam_q, gate_q))
        gate_normal_vec = cr.qv_mult(gate_global_q, [0, 0, 1])
        heading_to_gate = math.atan2((gate_global_p[1]-bebop_pos[1]),gate_global_p[0]-bebop_pos[0])
        heading_of_gate = math.atan2(gate_normal_vec[1],gate_normal_vec[0])
        heading_difference = math.fabs(heading_to_gate - heading_of_gate)*180/math.pi

        rospy.loginfo("gate_global_p")
        rospy.loginfo(gate_global_p)
        rospy.loginfo("gate_global_q")
        rospy.loginfo(gate_global_q)
        rospy.loginfo("gate_normal_vec")
        rospy.loginfo(gate_normal_vec)

        if 90 < heading_difference < 270:
            if heading_of_gate < 0:
                heading_of_gate = heading_of_gate + math.pi
            else:
                heading_of_gate = heading_of_gate - math.pi

        gate_global_p = [gate_global_p[0][0], gate_global_p[1][0], gate_global_p[2][0]]
        current_wp = cr.WP(gate_global_p, heading_of_gate)

        rospy.loginfo("current_wp")
        rospy.loginfo(current_wp)

        distance = 999
        if wp_visual_history is not None:
            distance = cr.length(gate_global_p - wp_visual_history[-1].pos)

        if distance > 0.5:
            wp_visual_history = [current_wp]
        else:
            wp_visual_history.append(current_wp)
            if len(wp_visual_history) > 10:
                del wp_visual_history[0]

        rospy.loginfo("distance")
        rospy.loginfo(distance)

        wp_visual = cr.find_average(wp_visual_history)

    rospy.loginfo("wp_visual")
    rospy.loginfo(wp_visual)

    msg = WP_Msg()
    if wp_visual is not None:
        msg.pos.x = wp_visual.pos[0]
        msg.pos.y = wp_visual.pos[1]
        msg.pos.z = wp_visual.pos[2]
        msg.hdg = wp_visual.hdg
    wp_visual_publisher.publish(msg)

    if wp_visual is not None:
        log_string = str(current_wp.pos[0]) + ", " + \
                     str(current_wp.pos[1]) + ", " + \
                     str(current_wp.pos[2]) + ", " + \
                     str(current_wp.hdg) + ", " + \
                     str(msg.pos.x) + ", " + \
                     str(msg.pos.y) + ", " + \
                     str(msg.pos.z) + ", " + \
                     str(msg.hdg) + ", " + \
                     str(bebop_pos[0][0]) + ", " + \
                     str(bebop_pos[1][0]) + ", " + \
                     str(bebop_pos[2][0]) + ", " + \
                     str(bebop_q[3]) + ", " + \
                     str(bebop_q[0]) + ", " + \
                     str(bebop_q[1]) + ", " + \
                     str(bebop_q[2]) + ", " + \
                     str(heading_to_gate)
    else:
        log_string = "0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0"

    visual_log_publisher.publish(log_string)

    return wp_visual, wp_visual_temp, wp_visual_history


def calculate_blind_wp(wp_blind, wp_blind_old, wp_visual, wp_visual_old, wp_look):
    global wp_blind_takeoff_time
    wp_blind_temp = wp_blind
    if state_auto == 4:
        rospy.loginfo("state 4")
        if wp_blind is None:
            if wp_blind_takeoff_time is None:
                rospy.loginfo("start timer")
                wp_blind_takeoff_time = time.time()
            elif wp_blind_takeoff_time + 3 < time.time():

                rospy.loginfo("set fake target")

                # implementation exactly reverse as in matlab. Invert necessary when not required in matlab vice versa
                bebop_pos = [odometry_merged.pose.pose.position.x, odometry_merged.pose.pose.position.y, odometry_merged.pose.pose.position.z]
                bebop_q = [odometry_merged.pose.pose.orientation.x, odometry_merged.pose.pose.orientation.y, odometry_merged.pose.pose.orientation.z,
                     odometry_merged.pose.pose.orientation.w]

                dx0 = [1, 0, 0]
                dx = cr.qv_mult(bebop_q, dx0)

                old_heading = math.atan2(dx[1], dx[0])
                new_heading = old_heading - 0*math.pi/4

                if new_heading > math.pi:
                    new_heading = new_heading - 2*math.pi
                elif new_heading < -math.pi:
                    new_heading = new_heading + 2 * math.pi

                blind_position = [2.0, -1.0, 1.0] #1.3
                blind_position_global = cr.qv_mult(bebop_q, blind_position) + bebop_pos
                blind_position_global = blind_position_global.tolist()

                look_position = [-5, 5, 0] # front, left, up
                look_position_global = cr.qv_mult(bebop_q, look_position) + bebop_pos
                look_position_global = look_position_global.tolist()

                # initialize with this position
                wp_blind = cr.WP(blind_position_global, new_heading)
                rospy.loginfo("global position set for blind")

                wp_look = cr.WP(look_position_global, 0)

    if state_auto == 5 and wp_visual is None:
        if wp_visual_old is not None:
            rospy.loginfo("state 5, set visual as blind")
            wp_blind = wp_visual_old

    if state_auto == 6 and wp_blind is None:
        rospy.loginfo("state 6, set new blind")
        # continue in gate direction for 0.5m

        extra_distance = 0.6 * np.array([[math.cos(wp_blind_old.hdg)], [math.sin(wp_blind_old.hdg)], [0]])
        wp_blind = cr.WP(wp_blind_old.pos + extra_distance, wp_blind_old.hdg)

        extra_distance = 10 * np.array([[math.cos(wp_blind_old.hdg)], [math.sin(wp_blind_old.hdg)], [0]])
        wp_look = cr.WP(wp_blind_old.pos + extra_distance, wp_blind_old.hdg)

        rospy.loginfo("land at: " + str(wp_blind))

    rospy.loginfo("wp_blind")
    rospy.loginfo(wp_blind)

    msg = WP_Msg()
    if wp_blind is not None:
        msg.pos.x = wp_blind.pos[0]
        msg.pos.y = wp_blind.pos[1]
        msg.pos.z = wp_blind.pos[2]
        msg.hdg = wp_blind.hdg
    wp_blind_publisher.publish(msg)

    msg = WP_Msg()
    if wp_look is not None:
        msg.pos.x = wp_look.pos[0]
        msg.pos.y = wp_look.pos[1]
    wp_look_publisher.publish(msg)

    return wp_blind, wp_blind_temp, wp_look


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


def navigate_throu(odometry_merged, wp):

    # implementation exactly reverse as in matlab. Invert necessary when not required in matlab vice versa
    bebop_q = [odometry_merged.pose.pose.orientation.x, odometry_merged.pose.pose.orientation.y, odometry_merged.pose.pose.orientation.z, odometry_merged.pose.pose.orientation.w]
    bebop_x_vec = cr.qv_mult(bebop_q, [1, 0, 0])
    hdg = math.atan2(bebop_x_vec[1], bebop_x_vec[0])

    rospy.loginfo("fly from")
    rospy.loginfo([odometry_merged.pose.pose.position.x, odometry_merged.pose.pose.position.y, odometry_merged.pose.pose.position.z, hdg])
    rospy.loginfo("fly to")
    rospy.loginfo(wp)

    quat = odometry_merged.pose.pose.orientation
    angle = tfs.euler_from_quaternion(bebop_q)[2]

    global_vel = odometry_merged.twist.twist.linear
    velocity = [global_vel.x*math.cos(angle) - global_vel.y*math.sin(angle),
                +global_vel.y*math.cos(angle) + global_vel.x*math.sin(angle),
                global_vel.z]

    diff_global = wp.pos - [odometry_merged.pose.pose.position.x, odometry_merged.pose.pose.position.y, odometry_merged.pose.pose.position.z]

    dist = math.hypot(diff_global[0], diff_global[1])

    gate_theta = wp.hdg
    pos_theta = math.atan2(diff_global[1], diff_global[0])

    d_theta = gate_theta-pos_theta
    if d_theta > math.pi:
        d_theta = -2*math.pi+d_theta
    elif d_theta < -math.pi:
        d_theta = 2*math.pi-d_theta
    else:
        pass

    y_pos_error = -dist * math.sin(d_theta)    
    y_vel_des = nav_throu_PID_y_pos.update(y_pos_error)

    x_pos_error = cr.min_value(dist * math.cos(d_theta), 0.15)
    x_vel_des = x_pos_error

    if abs(.4 * x_pos_error) ** 3 + .5 < y_pos_error:
        x_vel_des = 0        

    z_error = diff_global[2]
    
    r_error = -(angle - pos_theta)
    if r_error > math.pi:
        r_error = -2*math.pi+r_error
    elif r_error < -math.pi:
        r_error = 2*math.pi-r_error


    y_vel_error = cr.limit_value(sum(y_vel_des), 0.5) - velocity[1]
    x_vel_error = x_vel_des-velocity[0]

    nav_cmd_x = nav_throu_PID_x.update(x_vel_error)
    nav_cmd_y = nav_throu_PID_y.update(y_vel_error)
    nav_cmd_z = nav_throu_PID_z.update(z_error)
    nav_cmd_r = nav_throu_PID_r.update(r_error)


    msg = Auto_Driving_Msg()
    msg.x = cr.limit_value(sum(nav_cmd_x)+0.04, nav_limit_x)
    msg.y = cr.limit_value(sum(nav_cmd_y), nav_limit_y)
    msg.z = cr.limit_value(sum(nav_cmd_z), nav_limit_z)
    msg.r = cr.limit_value(sum(nav_cmd_r), nav_limit_r)

    log_string = str(d_theta) + ", " + \
                 str(x_vel_des) + ", " + \
                 str(velocity[0]) + ", " + \
                 str(x_vel_error) + ", " + \
                 str(nav_cmd_x[0]) + ", " + \
                 str(nav_cmd_x[2]) + ", " + \
                 str(sum(nav_cmd_x)) + ", " + \
                 str(msg.x) + ", " + \
                 str(d_theta) + ", " + \
                 str(y_pos_error) + ", " + \
                 str(y_vel_des[0]) + ", " + \
                 str(y_vel_des[2]) + ", " + \
                 str(sum(y_vel_des)) + ", " + \
                 str(velocity[1]) + ", " + \
                 str(y_vel_error) + ", " + \
                 str(nav_cmd_y[0]) + ", " + \
                 str(nav_cmd_y[2]) + ", " + \
                 str(sum(nav_cmd_y)) + ", " + \
                 str(msg.y) + ", " + \
                 str(diff_global[2]) + ", " + \
                 str(z_error) + ", " + \
                 str(nav_cmd_z[0]) + ", " + \
                 str(nav_cmd_z[2]) + ", " + \
                 str(sum(nav_cmd_z)) + ", " + \
                 str(msg.z) + ", " + \
                 str(pos_theta) + ", " + \
                 str(angle) + ", " + \
                 str(r_error) + ", " + \
                 str(nav_cmd_r[0]) + ", " + \
                 str(nav_cmd_r[2]) + ", " + \
                 str(sum(nav_cmd_r)) + ", " + \
                 str(msg.r)
    nav_log_publisher.publish(log_string)

    return msg
    # rospy.loginfo("calculated")
    # rospy.loginfo(auto_driving_msg)


def navigate_point(odometry_merged, wp, wp_look):

    # implementation exactly reverse as in matlab. Invert necessary when not required in matlab vice versa
    bebop_q = [odometry_merged.pose.pose.orientation.x, odometry_merged.pose.pose.orientation.y, odometry_merged.pose.pose.orientation.z, odometry_merged.pose.pose.orientation.w]
    bebop_x_vec = cr.qv_mult(bebop_q, [1, 0, 0])
    hdg = math.atan2(bebop_x_vec[1], bebop_x_vec[0])

    rospy.loginfo("fly from")
    rospy.loginfo([odometry_merged.pose.pose.position.x, odometry_merged.pose.pose.position.y, odometry_merged.pose.pose.position.z, hdg])
    rospy.loginfo("fly to")
    rospy.loginfo(wp)

    global_vel = odometry_merged.twist.twist.linear

    diff_global = wp.pos - [odometry_merged.pose.pose.position.x, odometry_merged.pose.pose.position.y, odometry_merged.pose.pose.position.z]
    diff_global_look = wp_look.pos - [odometry_merged.pose.pose.position.x, odometry_merged.pose.pose.position.y, odometry_merged.pose.pose.position.z]

    angle = tfs.euler_from_quaternion(bebop_q)[2]
    pos_theta = math.atan2(diff_global_look[1], diff_global_look[0])
    
    r_error = -(angle - pos_theta)
    if r_error > math.pi:
        r_error = -2*math.pi+r_error
    elif r_error < -math.pi:
        r_error = 2*math.pi-r_error

    x_pos_error = diff_global[0]
    y_pos_error = diff_global[1]
    
    x_vel_des = nav_point_PID_x_pos.update(x_pos_error)
    y_vel_des = nav_point_PID_y_pos.update(y_pos_error)

    x_vel_error = cr.limit_value(sum(x_vel_des), 0.5) - global_vel.x
    y_vel_error = cr.limit_value(sum(y_vel_des), 0.5) - global_vel.y

    z_error = diff_global[2]
    
    nav_cmd_x = nav_point_PID_x.update(x_vel_error)
    nav_cmd_y = nav_point_PID_y.update(y_vel_error)
    nav_cmd_z = nav_point_PID_z.update(z_error)
    nav_cmd_r = nav_point_PID_r.update(r_error)

    nav_cmd_x_veh = sum(nav_cmd_x)*math.cos(-angle) - sum(nav_cmd_y)*math.sin(-angle)
    nav_cmd_y_veh = sum(nav_cmd_y)*math.cos(-angle) + sum(nav_cmd_x)*math.sin(-angle)

    msg = Auto_Driving_Msg()
    msg.x = cr.limit_value(nav_cmd_x_veh, nav_limit_x)
    msg.y = cr.limit_value(nav_cmd_y_veh, nav_limit_y)
    msg.z = cr.limit_value(sum(nav_cmd_z), nav_limit_z)
    msg.r = cr.limit_value(sum(nav_cmd_r), nav_limit_r)

    log_string = str(x_pos_error) + ", " + \
                 str(x_vel_des[0]) + ", " + \
                 str(x_vel_des[2]) + ", " + \
                 str(sum(x_vel_des)) + ", " + \
                 str(global_vel.x) + ", " + \
                 str(x_vel_error) + ", " + \
                 str(nav_cmd_x[0]) + ", " + \
                 str(nav_cmd_x[2]) + ", " + \
                 str(sum(nav_cmd_x)) + ", " + \
                 str(msg.x) + ", " + \
                 str(y_pos_error) + ", " + \
                 str(y_vel_des[0]) + ", " + \
                 str(y_vel_des[2]) + ", " + \
                 str(sum(y_vel_des)) + ", " + \
                 str(global_vel.y) + ", " + \
                 str(y_vel_error) + ", " + \
                 str(nav_cmd_y[0]) + ", " + \
                 str(nav_cmd_y[2]) + ", " + \
                 str(sum(nav_cmd_y)) + ", " + \
                 str(msg.y) + ", " + \
                 str(diff_global[2]) + ", " + \
                 str(z_error) + ", " + \
                 str(nav_cmd_z[0]) + ", " + \
                 str(nav_cmd_z[2]) + ", " + \
                 str(sum(nav_cmd_z)) + ", " + \
                 str(msg.z) + ", " + \
                 str(pos_theta) + ", " + \
                 str(angle) + ", " + \
                 str(r_error) + ", " + \
                 str(nav_cmd_r[0]) + ", " + \
                 str(nav_cmd_r[2]) + ", " + \
                 str(sum(nav_cmd_r)) + ", " + \
                 str(msg.r)
    nav_log_publisher.publish(log_string)

    return msg
    # rospy.loginfo("calculated")
    # rospy.loginfo(auto_driving_msg)


def state_machine_advancement(state_auto, state_bebop, navigation_distance, state_auto_publisher, navigation_active, wp_blind, wp_visual, wp_look):
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
        navigation_active = "point"
        rospy.loginfo("takeoff completed")
        state_auto_publisher.publish(state_auto + 1)
    elif state_auto == 4 and wp_visual is not None: # we detected the gate and try to get there
        navigation_active = "throu"
        wp_blind = None
        wp_look = None
        rospy.loginfo("visually detected gate")
        state_auto_publisher.publish(state_auto + 1)
    elif state_auto == 4 and navigation_distance < 0.1: # we reached the blind point and can't see the gate -> land
        wp_blind = None
        wp_look = None
        navigation_active = "off"
        rospy.loginfo("can't change to visual. Landing")
        state_auto_publisher.publish(7)
    elif state_auto == 5 and navigation_distance < 0.2:        # drone reached gate, continue in gate direction
        navigation_active = "point"
        wp_blind = None
        rospy.loginfo("gate reached")
        state_auto_publisher.publish(state_auto + 1)
    elif state_auto == 6 and navigation_distance < 0.2:        # drone reached landing location
        wp_blind = None
        wp_look = None
        navigation_active = "off"
        rospy.loginfo("mission finished")
        state_auto_publisher.publish(state_auto + 1)
    elif state_auto == 7 and state_bebop == 4:      # drone initiated landing
        rospy.loginfo("landing started")
        state_auto_publisher.publish(state_auto + 1)
    elif state_auto == 8 and state_bebop == 0:      # drone was landing and has landed
        rospy.loginfo("landing completed")
        state_auto_publisher.publish(state_auto + 1)

    return [navigation_active, wp_blind, wp_look]


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
    wp_blind_takeoff_time = None
    wp_blind = None
    wp_blind_old = None
    wp_look = None
    wp = None
    navigation_active = "off"
    nav_point_PID_x_pos = cr.PID2(.7, 0, 2.5)
    nav_point_PID_y_pos = cr.PID2(.7, 0, 2.5)
    nav_point_PID_x     = cr.PID2(0.1, 0, 1.0)
    nav_point_PID_y     = cr.PID2(0.1, 0, 1.0)
    nav_point_PID_z     = cr.PID(1.0, 0, 0.0)
    nav_point_PID_r     = cr.PID(0.5, 0, 1.0)
    nav_throu_PID_y_pos = cr.PID2(.7, 0, 3)
    nav_throu_PID_x     = cr.PID(0.07, 0, 0.0)
    nav_throu_PID_y     = cr.PID2(0.12, 0, 1.0)
    nav_throu_PID_z     = cr.PID(1.0, 0, 0.0)
    nav_throu_PID_r     = cr.PID(1.2, 0, 1.0)
    nav_limit_x = .05 #.25
    nav_limit_y = .15 #.4
    nav_limit_z = .1 #.75
    nav_limit_r = .5 #1
    empty_command = True
    auto_driving_msg = Auto_Driving_Msg()
    min_distance = 999

    # Publishers
    state_auto_publisher = rospy.Publisher("/auto/state_auto",        Int32,              queue_size=1, latch=True)
    auto_drive_publisher = rospy.Publisher("/auto/auto_drive",        Auto_Driving_Msg,   queue_size=1, latch=True)
    wp_visual_publisher  = rospy.Publisher("/auto/wp_visual",         WP_Msg,             queue_size=1, latch=True)
    wp_blind_publisher   = rospy.Publisher("/auto/wp_blind",          WP_Msg,             queue_size=1, latch=True)
    wp_look_publisher    = rospy.Publisher("/auto/wp_look",           WP_Msg,             queue_size=1, latch=True)
    nav_log_publisher    = rospy.Publisher("/auto/navigation_logger", String,             queue_size=1, latch=True)
    visual_log_publisher = rospy.Publisher("/auto/visual_logger",     String,             queue_size=1, latch=True)

    # Subscribers
    rospy.Subscriber("/bebop/states/ardrone3/PilotingState/FlyingStateChanged", Ardrone3PilotingStateFlyingStateChanged, callback_states_changed, "state_bebop")
    rospy.Subscriber("/auto/state_auto", Int32, callback_states_changed, "state_auto")
    rospy.Subscriber("/bebop/odom", Odometry, callback_odometry_merged_changed)
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

    rospy.spin()
