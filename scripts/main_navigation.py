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
from std_msgs.msg import Int32, String, Float64MultiArray, Bool, Float32
from bebop_msgs.msg import Ardrone3PilotingStateFlyingStateChanged
from bebop_auto.msg import Auto_Driving_Msg, Gate_Detection_Msg, WP_Msg
from nav_msgs.msg import Odometry
from tf import transformations as tfs
import common_resources as cr


def signal_handler():
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


def callback_visual_gate_dynamic_changed(input_data):
    global detection_dynamic_input_history
    global detection_dynamic_data

    measurement = input_data.data

    rospy.loginfo("visual hand detected")

    if detection_dynamic_data.period is None:
        detection_dynamic_input_history = np.append(detection_dynamic_input_history,
                                                    [[measurement[0]], [measurement[1]]], axis=1)
        if detection_dynamic_input_history.shape[0] > 20:
            rospy.loginfo("enough measurements")
            detection_dynamic_input_history = np.delete(detection_dynamic_input_history, 0, axis=1)
            # calculate std deviation of list
            periods = cr.calculate_periods(detection_dynamic_input_history)
            std_deviation = np.std(periods)
            # when std dev is low enough, provide waypoint
            if std_deviation < 0.2:
                rospy.loginfo("measurements accepted")
                detection_dynamic_data.period = np.mean(periods)
                rospy.loginfo(std_deviation)
            else:
                rospy.loginfo("standard deviation too high:")
                rospy.loginfo(std_deviation)
        else:
            rospy.loginfo("collecting measurements")
    else:
        # add to list only if gate position is close to where it's supposed to be
        t_diff = np.array(measurement[1]) - detection_dynamic_input_history[0][-1]
        angle_theory = t_diff / (2*math.pi*detection_dynamic_data.period) + detection_dynamic_input_history[1][-1]
        angle_theory = angle_theory % (2 * math.pi)
        diff = math.fabs(angle_theory - measurement[0])
        diff = min(diff, 2 * math.pi - diff)
        if diff < 20 * math.pi / 180:
            rospy.loginfo("use detected pointer")
            detection_dynamic_input_history = np.append(detection_dynamic_input_history,
                                                        [[measurement[0]], [measurement[1]]], axis=1)
            detection_dynamic_input_history = np.delete(detection_dynamic_input_history, 0, axis=1)
            periods = cr.calculate_periods(detection_dynamic_input_history)
            detection_dynamic_data.period = np.mean(periods)

            # calculate current pointer position based on 5 last measurements
            t_delta = time.time() - detection_dynamic_input_history[0][-5:]
            a_delta = t_delta / (2*math.pi*detection_dynamic_data.period)
            angles = a_delta + detection_dynamic_input_history[1][-5:]
            angles = angles % (2 * math.pi)

            current_angle = math.atan2(np.sum(np.sin(angles)), np.sum(np.cos(angles)))
            if current_angle < 0:
                current_angle = current_angle + math.pi
            detection_dynamic_data.theta = current_angle
            rospy.loginfo("detection_dynamic_data.theta")
            rospy.loginfo(current_angle)
        else:
            rospy.loginfo("discard detected pointer")

    rospy.loginfo("visual hand period")
    rospy.loginfo(detection_dynamic_data.period)


def callback_visual_gate_detection_changed(data):
    global wp_average
    global wp_input_history

    if not detection_active:
        rospy.loginfo("detection not active")
        log_string = "0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0"
        publisher_visual_log.publish(log_string)
        return

    if data.tvec == ():
        rospy.loginfo("empty visual input")
        log_string = "0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0"
        publisher_visual_log.publish(log_string)
        return

    rospy.loginfo("visual gate detected")

    # read data
    bebop_position = data.bebop_pose.position
    bebop_orientation = data.bebop_pose.orientation

    # bebop position and orientation
    bebop_p = np.array([bebop_position.x, bebop_position.y, bebop_position.z])
    bebop_q = np.array([bebop_orientation.x, bebop_orientation.y, bebop_orientation.z, bebop_orientation.w])

    # gate position and orientation
    # rospy.loginfo("tvec")
    # rospy.loginfo(data.tvec)
    # rospy.loginfo("rvec")
    # rospy.loginfo(data.rvec)

    gate_pos = np.array(data.tvec)
    gate_q = cr.axang2quat(np.array(data.rvec))
    #
    # rospy.loginfo("bebop_p")
    # rospy.loginfo(bebop_p)
    # rospy.loginfo("bebop_q")
    # rospy.loginfo(bebop_q)
    # rospy.loginfo("gate_pos")
    # rospy.loginfo(gate_pos)
    # rospy.loginfo("gate_q")
    # rospy.loginfo(gate_q)

    gate_global_p = cr.qv_mult(bebop_q, cr.qv_mult(cr.cam_q, gate_pos) + cr.BZ) + bebop_p
    gate_global_q = tfs.quaternion_multiply(bebop_q, tfs.quaternion_multiply(cr.cam_q, gate_q))
    gate_normal_vec = cr.qv_mult(gate_global_q, [0, 0, 1])
    heading_to_gate = math.atan2((gate_global_p[1] - bebop_p[1]), gate_global_p[0] - bebop_p[0])
    heading_of_gate = math.atan2(gate_normal_vec[1], gate_normal_vec[0])
    heading_difference = math.fabs(heading_to_gate - heading_of_gate) * 180 / math.pi
    #
    # rospy.loginfo("gate_global_p")
    # rospy.loginfo(gate_global_p)
    # rospy.loginfo("gate_global_q")
    # rospy.loginfo(gate_global_q)
    # rospy.loginfo("gate_normal_vec")
    # rospy.loginfo(gate_normal_vec)

    if 90 < heading_difference < 270:
        if heading_of_gate < 0:
            heading_of_gate = heading_of_gate + math.pi
        else:
            heading_of_gate = heading_of_gate - math.pi

    wp_current = cr.WP(gate_global_p, heading_of_gate)
    msg = WP_Msg()
    msg.pos.x = wp_current.pos[0]
    msg.pos.y = wp_current.pos[1]
    msg.pos.z = wp_current.pos[2]
    msg.hdg = wp_current.hdg
    publisher_wp_current.publish(msg)

    rospy.loginfo("new measured wp_current")
    rospy.loginfo(wp_current)

    # while there is no average yet: start collecting gate positions and evaluate standard deviation
    if wp_average is None:
        wp_input_history.append(wp_current)
        if len(wp_input_history) > 10:
            rospy.loginfo("enough measurements")
            del wp_input_history[0]
            # calculate std deviation of list
            average = cr.find_average(wp_input_history)
            std_deviation = cr.find_std_dev_waypoints(average, wp_input_history)
            # when std dev is low enough, provide waypoint
            if std_deviation < 0.4:
                rospy.loginfo("measurements accepted")
                wp_average = average
            else:
                rospy.loginfo("standard deviation too high:")
                rospy.loginfo(std_deviation)
        else:
            std_deviation = 5
            average = cr.WP([0, 0, 0], 0)
            rospy.loginfo("collecting measurements")

        log_string = str(wp_current.pos[0]) + ", " + \
            str(wp_current.pos[1]) + ", " + \
            str(wp_current.pos[2]) + ", " + \
            str(wp_current.hdg) + ", " + \
            str(average.pos[0]) + ", " + \
            str(average.pos[1]) + ", " + \
            str(average.pos[2]) + ", " + \
            str(average.hdg) + ", " + \
            str(bebop_p[0]) + ", " + \
            str(bebop_p[1]) + ", " + \
            str(bebop_p[2]) + ", " + \
            str(bebop_q[3]) + ", " + \
            str(bebop_q[0]) + ", " + \
            str(bebop_q[1]) + ", " + \
            str(bebop_q[2]) + ", " + \
            str(heading_to_gate) + ", " + \
            str(std_deviation) + ", " + \
            str(1.0)
    else:
        # now, add to list only if gate position is close to last one
        distance = np.linalg.norm(np.array(gate_global_p) - wp_input_history[-1].pos)
        if distance < 1:
            rospy.loginfo("use detected gate")
            wp_input_history.append(wp_current)
            del wp_input_history[0]
            wp_average = cr.find_average(wp_input_history)
            rospy.loginfo("wp_average")
            rospy.loginfo(wp_average)
        else:
            print("discard detected gate")

        log_string = str(wp_current.pos[0]) + ", " + \
            str(wp_current.pos[1]) + ", " + \
            str(wp_current.pos[2]) + ", " + \
            str(wp_current.hdg) + ", " + \
            str(wp_average.pos[0]) + ", " + \
            str(wp_average.pos[1]) + ", " + \
            str(wp_average.pos[2]) + ", " + \
            str(wp_average.hdg) + ", " + \
            str(bebop_p[0]) + ", " + \
            str(bebop_p[1]) + ", " + \
            str(bebop_p[2]) + ", " + \
            str(bebop_q[3]) + ", " + \
            str(bebop_q[0]) + ", " + \
            str(bebop_q[1]) + ", " + \
            str(bebop_q[2]) + ", " + \
            str(heading_to_gate) + ", " + \
            str(distance) + ", " + \
            str(2.0)

    publisher_visual_log.publish(log_string)

    msg = WP_Msg()
    if wp_average is not None:
        msg.pos.x = wp_average.pos[0]
        msg.pos.y = wp_average.pos[1]
        msg.pos.z = wp_average.pos[2]
        msg.hdg = wp_average.hdg
    publisher_wp_average.publish(msg)

    rospy.loginfo("visual gate done")

    return


def calculate_visual_wp():
    global wp_visual
    global wp_average
    global wp_visual_old
    global wp_look

    wp_visual_old = wp_visual or wp_visual_old

    if state_auto == 72:
        rospy.loginfo("state 72, calculate hover position")

        # hover 4m behind dynamic gate at -0.5 height
        gate_pos = wp_average.pos
        gate_heading = wp_average.hdg
        hover_distance = -3.2
        hover_alt = -0.5
        extra_dist = np.array([hover_distance*math.cos(gate_heading), hover_distance*math.sin(gate_heading), hover_alt])
        wp_visual = cr.WP(gate_pos + extra_dist, gate_heading)

        wp_look = wp_average

    else:
        wp_visual = wp_average

    rospy.loginfo("wp_visual")
    rospy.loginfo(wp_visual)

    msg = WP_Msg()
    if wp_visual is not None:
        msg.pos.x = wp_visual.pos[0]
        msg.pos.y = wp_visual.pos[1]
        msg.pos.z = wp_visual.pos[2]
        msg.hdg = wp_average.hdg
    publisher_wp_visual.publish(msg)
    return


def calculate_blind_wp():
    global wp_blind
    global wp_blind_old
    global wp_visual
    global wp_visual_old
    global wp_average
    global wp_input_history
    global wp_look
    global wp_blind_takeoff_time

    bebop_position = bebop_odometry.pose.pose.position
    bebop_orientation = bebop_odometry.pose.pose.orientation

    wp_blind_old = wp_blind or wp_blind_old

    if state_auto == 5 and wp_blind is None:
        rospy.loginfo("state 5: set fake start location to see first gate")

        bebop_p = [bebop_position.x, bebop_position.y, bebop_position.z]
        bebop_q = [bebop_orientation.x, bebop_orientation.y, bebop_orientation.z, bebop_orientation.w]

        blind_position = [0.0, 0.0, 0.0]  # front, left , up
        blind_position_global = cr.qv_mult(bebop_q, blind_position) + bebop_p
        blind_position_global = blind_position_global.tolist()

        look_position = [5, 0, 0]  # front, left, up
        look_position_global = cr.qv_mult(bebop_q, look_position) + bebop_p
        look_position_global = look_position_global.tolist()

        # initialize with this position
        wp_blind = cr.WP(blind_position_global, 0)
        rospy.loginfo("global position set for blind")

        wp_look = cr.WP(look_position_global, 0)

    if state_auto == 7 and wp_blind is None:
        rospy.loginfo("state 7, set new blind at 90deg")
        # continue in gate direction for 0.5m

        fly_start = wp_visual_old.pos
        fly_distance = 0.5
        fly_heading = wp_visual_old.hdg
        fly_vec = fly_distance * np.array([math.cos(fly_heading), math.sin(fly_heading), 0])
        wp_blind = cr.WP(fly_start + fly_vec, 0)

        look_start = wp_visual_old.pos
        look_distance = 10
        look_heading = wp_visual_old.hdg - math.pi/2
        look_vec = look_distance * np.array([math.cos(look_heading), math.sin(look_heading), 0])
        wp_look = cr.WP(look_start.pos + look_vec, 0)

        wp_visual = None
        wp_average = None
        wp_input_history = []

        rospy.loginfo("fly to turn: " + str(wp_blind))

    if state_auto == 8 and wp_blind is None:
        rospy.loginfo("state 8, set new blind at 180deg")
        # continue in gate direction for 0.5m

        fly_start = wp_visual_old.pos
        fly_distance = 1.0
        fly_heading = wp_visual_old.hdg
        fly_vec = fly_distance * np.array([math.cos(fly_heading), math.sin(fly_heading), 0])
        wp_blind = cr.WP(fly_start + fly_vec, 0)

        look_start = wp_visual_old.pos
        look_distance = 10
        look_heading = wp_visual_old.hdg - math.pi
        look_vec = look_distance * np.array([math.cos(look_heading), math.sin(look_heading), 0])
        wp_look = cr.WP(look_start.pos + look_vec, 0)

        wp_visual = None
        wp_average = None
        wp_input_history = []

        rospy.loginfo("fly to turn2: " + str(wp_blind))

    if state_auto == 10 and wp_blind is None:
        rospy.loginfo("state 10, set new blind at ldg")
        # continue in gate direction for 0.5m

        fly_start = wp_visual_old.pos
        fly_distance = 0.5
        fly_heading = wp_visual_old.hdg
        fly_vec = fly_distance * np.array([math.cos(fly_heading), math.sin(fly_heading), 0])
        wp_blind = cr.WP(fly_start + fly_vec, 0)

        look_start = wp_visual_old.pos
        look_distance = 10
        look_heading = wp_visual_old.hdg
        look_vec = look_distance * np.array([math.cos(look_heading), math.sin(look_heading), 0])
        wp_look = cr.WP(look_start.pos + look_vec, 0)

        wp_visual = None
        wp_average = None
        wp_input_history = []

        rospy.loginfo("fly to turn2: " + str(wp_blind))


    if state_auto == 71 and wp_blind is None:
        rospy.loginfo("state 71, set new blind at to reach gate")

        bebop_p = [bebop_position.x, bebop_position.y, bebop_position.z]
        bebop_q = [bebop_orientation.x, bebop_orientation.y, bebop_orientation.z, bebop_orientation.w]

        blind_position = [0.0, 0.0, 0.5]  # front, left , up
        blind_position_global = cr.qv_mult(bebop_q, blind_position) + bebop_p
        blind_position_global = blind_position_global.tolist()

        look_position = [5, 0, 0]  # front, left, up
        look_position_global = cr.qv_mult(bebop_q, look_position) + bebop_p
        look_position_global = look_position_global.tolist()

        # initialize with this position
        wp_blind = cr.WP(blind_position_global, 0)
        rospy.loginfo("global position set for blind")

        wp_look = cr.WP(look_position_global, 0)


    rospy.loginfo("wp_blind")
    rospy.loginfo(wp_blind)

    msg = WP_Msg()

    if wp_blind is not None:
        msg.pos.x = wp_blind.pos[0]
        msg.pos.y = wp_blind.pos[1]
        msg.pos.z = wp_blind.pos[2]
        msg.hdg = wp_blind.hdg
    publisher_wp_blind.publish(msg)

    msg = WP_Msg()
    if wp_look is not None:
        msg.pos.x = wp_look.pos[0]
        msg.pos.y = wp_look.pos[1]
    publisher_wp_look.publish(msg)

    return


def select_waypoint():
    global wp_select
    if wp_visual is not None:
        rospy.loginfo("fly visual")
        wp_select = wp_visual
    elif wp_blind is not None:
        rospy.loginfo("fly blind")
        wp_select = wp_blind
    else:
        rospy.loginfo("no wp")
        wp_select = None


def navigate_through():
    bebop_position = bebop_odometry.pose.pose.position
    bebop_orientation = bebop_odometry.pose.pose.orientation

    bebop_p = [bebop_position.x, bebop_position.y, bebop_position.z]
    bebop_q = [bebop_orientation.x, bebop_orientation.y, bebop_orientation.z, bebop_orientation.w]
    bebop_x_vec = cr.qv_mult(bebop_q, [1, 0, 0])
    hdg = math.atan2(bebop_x_vec[1], bebop_x_vec[0])

    rospy.loginfo("fly from")
    rospy.loginfo([bebop_p[0], bebop_p[1], bebop_p[2], hdg])
    rospy.loginfo("fly to")
    rospy.loginfo(wp_select)

    # quat = bebop_odometry.pose.pose.orientation
    angle = tfs.euler_from_quaternion(bebop_q)[2]

    velocity = bebop_odometry.twist.twist.linear

    diff_global = wp_select.pos - bebop_p

    dist = math.hypot(diff_global[0], diff_global[1])

    gate_theta = wp_select.hdg
    pos_theta = math.atan2(diff_global[1], diff_global[0])

    d_theta = gate_theta - pos_theta
    if d_theta > math.pi:
        d_theta = -2 * math.pi + d_theta
    elif d_theta < -math.pi:
        d_theta = 2 * math.pi - d_theta
    else:
        pass

    y_pos_error = -dist * math.sin(d_theta)
    y_vel_des = nav_through_PID_y_pos.update(y_pos_error)

    x_pos_error = cr.min_value(dist * math.cos(d_theta), 0.15)
    x_vel_des = x_pos_error

    if abs(.5 * x_pos_error) ** 3 + .2 < y_pos_error:
        x_vel_des = 0

    z_error = diff_global[2]

    r_error = -(angle - pos_theta)
    if r_error > math.pi:
        r_error = -2 * math.pi + r_error
    elif r_error < -math.pi:
        r_error = 2 * math.pi - r_error

    y_vel_error = cr.limit_value(sum(y_vel_des), 0.2) - velocity.y
    x_vel_error = x_vel_des - velocity.x

    nav_cmd_x = nav_through_PID_x_vel.update(x_vel_error)
    nav_cmd_y = nav_through_PID_y_vel.update(y_vel_error)
    nav_cmd_z = nav_through_PID_z_vel.update(z_error)
    nav_cmd_r = nav_through_PID_r_vel.update(r_error)

    msg = Auto_Driving_Msg()
    msg.x = cr.limit_value(sum(nav_cmd_x) + 0.04, nav_limit_x)
    msg.y = cr.limit_value(sum(nav_cmd_y), nav_limit_y)
    msg.z = cr.limit_value(sum(nav_cmd_z), nav_limit_z)
    msg.r = cr.limit_value(sum(nav_cmd_r), nav_limit_r)

    log_string = str(
         dist) + ", " + str(
        0)+', ' + str(
        0)+', ' + str(
        0)+', ' + str(
        x_vel_des) + ", " + str(
        velocity.x) + ", " + str(
        x_vel_error) + ", " + str(
        nav_cmd_x[0]) + ", " + str(
        nav_cmd_x[1]) + ", " + str(
        nav_cmd_x[2]) + ", " + str(
        sum(nav_cmd_x)) + ", " + str(
        msg.x) + ", " + str(
        y_pos_error) + ", " + str(
        y_vel_des[0]) + ", " + str(
        y_vel_des[1]) + ", " + str(
        y_vel_des[2]) + ", " + str(
        sum(y_vel_des)) + ", " + str(
        velocity.y) + ", " + str(
        y_vel_error) + ", " + str(
        nav_cmd_y[0]) + ", " + str(
        nav_cmd_y[1]) + ", " + str(
        nav_cmd_y[2]) + ", " + str(
        sum(nav_cmd_y)) + ", " + str(
        msg.y) + ", " + str(
        diff_global[2]) + ", " + str(
        z_error) + ", " + str(
        nav_cmd_z[0]) + ", " + str(
        nav_cmd_z[1]) + ", " + str(
        nav_cmd_z[2]) + ", " + str(
        sum(nav_cmd_z)) + ", " + str(
        msg.z) + ", " + str(
        pos_theta) + ", " + str(
        angle) + ", " + str(
        r_error) + ", " + str(
        nav_cmd_r[0]) + ", " + str(
        nav_cmd_r[1]) + ", " + str(
        nav_cmd_r[2]) + ", " + str(
        sum(nav_cmd_r)) + ", " + str(
        msg.r) + ", " + str(
        0) + ", " + str(
        0) + ", " + str(
        0) + ", " + str(
        0) + ", " + str(
        0)

    publisher_nav_log.publish(log_string)

    return msg
    # rospy.loginfo("calculated")
    # rospy.loginfo(auto_driving_msg)


def navigate_point():
    bebop_position = bebop_odometry.pose.pose.position
    bebop_orientation = bebop_odometry.pose.pose.orientation

    bebop_p = [bebop_position.x, bebop_position.y, bebop_position.z]
    bebop_q = [bebop_orientation.x, bebop_orientation.y, bebop_orientation.z, bebop_orientation.w]
    bebop_x_vec = cr.qv_mult(bebop_q, [1, 0, 0])
    hdg = math.atan2(bebop_x_vec[1], bebop_x_vec[0])

    rospy.loginfo("fly from")
    rospy.loginfo(
        [bebop_odometry.pose.pose.position.x, bebop_odometry.pose.pose.position.y, bebop_odometry.pose.pose.position.z,
         hdg])
    rospy.loginfo("fly to")
    rospy.loginfo(wp_select)

    angle = tfs.euler_from_quaternion(bebop_q)[2]
    velocity = bebop_odometry.twist.twist.linear

    global_vel = [velocity.x * math.cos(angle) - velocity.y * math.sin(angle),
                  velocity.y * math.cos(angle) + velocity.x * math.sin(angle),
                  velocity.z]

    diff_global = wp_select.pos - bebop_p
    diff_global_look = wp_look.pos - bebop_p

    pos_theta = math.atan2(diff_global_look[1], diff_global_look[0])

    r_error = -(angle - pos_theta)
    if r_error > math.pi:
        r_error = -2 * math.pi + r_error
    elif r_error < -math.pi:
        r_error = 2 * math.pi - r_error

    x_pos_error = diff_global[0]
    y_pos_error = diff_global[1]

    x_vel_des = nav_point_PID_x_pos.update(x_pos_error)
    y_vel_des = nav_point_PID_y_pos.update(y_pos_error)

    x_vel_error = cr.limit_value(sum(x_vel_des), 0.1) - global_vel[0]
    y_vel_error = cr.limit_value(sum(y_vel_des), 0.1) - global_vel[1]

    z_error = diff_global[2]

    nav_cmd_x = nav_point_PID_x_vel.update(x_vel_error)
    nav_cmd_y = nav_point_PID_y_vel.update(y_vel_error)
    nav_cmd_z = nav_point_PID_z_vel.update(z_error)
    nav_cmd_r = nav_point_PID_r_vel.update(r_error)

    nav_cmd_x_veh = sum(nav_cmd_x) * math.cos(-angle) - sum(nav_cmd_y) * math.sin(-angle)
    nav_cmd_y_veh = sum(nav_cmd_y) * math.cos(-angle) + sum(nav_cmd_x) * math.sin(-angle)

    msg = Auto_Driving_Msg()
    msg.x = cr.limit_value(nav_cmd_x_veh, nav_limit_x)
    msg.y = cr.limit_value(nav_cmd_y_veh, nav_limit_y)
    msg.z = cr.limit_value(sum(nav_cmd_z), nav_limit_z)
    msg.r = cr.limit_value(sum(nav_cmd_r), nav_limit_r)

    log_string = str(
        x_pos_error) + ", " + str(
        x_vel_des[0]) + ", " + str(
        x_vel_des[1]) + ", " + str(
        x_vel_des[2]) + ", " + str(
        sum(x_vel_des)) + ", " + str(
        global_vel[0]) + ", " + str(
        x_vel_error) + ", " + str(
        nav_cmd_x[0]) + ", " + str(
        nav_cmd_x[1]) + ", " + str(
        nav_cmd_x[2]) + ", " + str(
        sum(nav_cmd_x)) + ", " + str(
        msg.x) + ", " + str(
        y_pos_error) + ", " + str(
        y_vel_des[0]) + ", " + str(
        y_vel_des[1]) + ", " + str(
        y_vel_des[2]) + ", " + str(
        sum(y_vel_des)) + ", " + str(
        global_vel[1]) + ", " + str(
        y_vel_error) + ", " + str(
        nav_cmd_y[0]) + ", " + str(
        nav_cmd_y[1]) + ", " + str(
        nav_cmd_y[2]) + ", " + str(
        sum(nav_cmd_y)) + ", " + str(
        msg.y) + ", " + str(
        diff_global[2]) + ", " + str(
        z_error) + ", " + str(
        nav_cmd_z[0]) + ", " + str(
        nav_cmd_z[1]) + ", " + str(
        nav_cmd_z[2]) + ", " + str(
        sum(nav_cmd_z)) + ", " + str(
        msg.z) + ", " + str(
        pos_theta) + ", " + str(
        angle) + ", " + str(
        r_error) + ", " + str(
        nav_cmd_r[0]) + ", " + str(
        nav_cmd_r[1]) + ", " + str(
        nav_cmd_r[2]) + ", " + str(
        sum(nav_cmd_r)) + ", " + str(
        msg.r) + ", " + str(
        0) + ", " + str(
        0) + ", " + str(
        0) + ", " + str(
        0) + ", " + str(
        0)
    publisher_nav_log.publish(log_string)

    return msg
    # rospy.loginfo("calculated")
    # rospy.loginfo(auto_driving_msg)


def navigate_dynamic():
    global detection_dynamic_data
    bebop_position = bebop_odometry.pose.pose.position
    bebop_orientation = bebop_odometry.pose.pose.orientation

    bebop_p = [bebop_position.x, bebop_position.y, bebop_position.z]
    bebop_q = [bebop_orientation.x, bebop_orientation.y, bebop_orientation.z, bebop_orientation.w]

    diff_global_look = wp_visual.pos - bebop_p
    angle = tfs.euler_from_quaternion(bebop_q)[2]
    pos_theta = math.atan2(diff_global_look[1], diff_global_look[0])
    r_error = angle - pos_theta
    if r_error > math.pi:
        r_error = -2 * math.pi + r_error
    elif r_error < -math.pi:
        r_error = 2 * math.pi - r_error

    msg = Auto_Driving_Msg()

    rospy.loginfo("DYN - dynamic state")
    rospy.loginfo(detection_dynamic_data.state)

    # stabilize
    if detection_dynamic_data.state == 0:
        # start a timer
        if detection_dynamic_data.timer != 0.0:
            # when timer has elapsed, check yaw error
            if time.time()-detection_dynamic_data.timer > .8:
                rospy.loginfo("DYN - check yaw error")
                if r_error < .08:
                    rospy.loginfo("DYN - yaw ok, state 2")
                    detection_dynamic_data.state = 2  # error is small -> wait for gate rotatiom
                else:
                    rospy.loginfo("DYN - yaw error high, state 1")
                    detection_dynamic_data.state = 1  # error is large -> correct yaw
                detection_dynamic_data.timer = 0.0
        else:
            detection_dynamic_data.timer = time.time()
            rospy.loginfo("DYN - timer started")

    # rotate
    elif detection_dynamic_data.state == 1:
        msg = Auto_Driving_Msg()
        # check error again before sending command
        if r_error < .08:
            rospy.loginfo("DYN - yaw ok, state 0")
            detection_dynamic_data.state = 0
        else:
            rospy.loginfo("DYN - yawing")
            msg.r = cr.limit_value(r_error, .1)

    # wait for gate rotation
    elif detection_dynamic_data.state == 2:
        if detection_dynamic_data.period is None:
            rospy.loginfo("DYN - no period yet, state 0")
            detection_dynamic_data.state = 0
        else:
            angle_difference = abs(detection_dynamic_data.theta-detection_dynamic_data.theta_trigger())
            if angle_difference < abs(2*math.pi/(detection_dynamic_data.period*5)*.7):
                rospy.loginfo("DYN - pointer angle triggered, state 3")
                detection_dynamic_data.timer = time.time()
                detection_dynamic_data.state = 3
            else:
                rospy.loginfo("DYN - wait for rotation")
                detection_dynamic_data.state = 5

    # full forward
    elif detection_dynamic_data.state == 3:
        if time.time()-detection_dynamic_data.timer > 1.5:
            rospy.loginfo("DYN - passed through, state 4")
            detection_dynamic_data.state = 4
            detection_dynamic_data.timer = time.time()
        else:
            rospy.loginfo("DYN - full forward")
            # msg.x = 1
            detection_dynamic_data.state = 5

    # pause
    elif detection_dynamic_data.state == 4:
        if time.time()-detection_dynamic_data.timer < 1.0:
            rospy.loginfo("DYN - break command")
            # send brake command
            pass
        else:
            # advance own state to advance state machine
            rospy.loginfo("DYN - completed")
            detection_dynamic_data.state = 5

    return msg


def calculate_distance():
    if wp_select is None or bebop_odometry is None:
        return 999

    bebop_position = bebop_odometry.pose.pose.position
    bebop_p = [bebop_position.x, bebop_position.y, bebop_position.z]
    diff_global = wp_select.pos - bebop_p

    if nav_active == "point":
        linear_distance = np.linalg.norm(diff_global)

        bebop_orientation = bebop_odometry.pose.pose.orientation
        bebop_q = np.array([bebop_orientation.x, bebop_orientation.y, bebop_orientation.z, bebop_orientation.w])
        own_heading = tfs.euler_from_quaternion(bebop_q)[2]
        diff_global_look = wp_look.pos - bebop_p
        pos_theta = math.atan2(diff_global_look[1], diff_global_look[0])
        angular_diff = -(own_heading - pos_theta)
        angular_diff = min(angular_diff, math.pi-angular_diff)
        return linear_distance + angular_diff * (180.0 / 20 * 0.3) / math.pi  # 20deg offset equal 30cm

    elif nav_active == "through":
        flat_distance = np.linalg.norm([diff_global[0], diff_global[1], 0])
        heading_to_gate = math.atan2(wp_select.pos[1] - bebop_position.y, wp_select.pos[0] - bebop_position.x)
        heading_of_gate = wp_select.hdg
        heading_difference = heading_to_gate - heading_of_gate
        return flat_distance * math.cos(heading_difference)


def state_machine_advancement(navigation_distance):
    global wp_blind
    global wp_average
    global nav_active
    global detection_active
    global wp_input_history
    global wp_blind_takeoff_time

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

    rospy.loginfo("state machine sees: auto " + str(state_auto) + " and bebop " + str(state_bebop))

    if state_auto == 2 and state_bebop == 1:                           # air received response from ground
        rospy.loginfo("drone takes off")
        publisher_state_auto.publish(state_auto + 1)                   # 3 - drone takes off

    elif state_auto == 3 and state_bebop == 2:                         # drone is hovering/flying
        wp_blind_takeoff_time = time.time()
        rospy.loginfo("drone hovers 3 sec")
        publisher_state_auto.publish(4)                                # 4 - drone hovers 3 sec

    elif state_auto == 4 and wp_blind_takeoff_time + 3 < time.time():  # drone has hovered for 3 sec
        publisher_gate_size.publish(2.1)
        detection_active = True
        nav_active = "point"
        rospy.loginfo("fly blind towards G1, with detection")
        # publisher_state_auto.publish(11)                             # 11 - fly blind towards G1, with detection
        publisher_state_auto.publish(71)                               # 71 - fly blind towards G7, with detection
        # publisher_state_auto.publish(70)                             # 70 - fly blind towards G7, without detection

    elif state_auto == 11 and wp_average is not None:                  # G1 detected
        nav_active = "through"
        wp_blind = None
        rospy.loginfo("fly visual to G1")
        publisher_state_auto.publish(state_auto + 1)                   # 12 - fly visual to G1

    elif state_auto == 12 and navigation_distance < 0.5:               # drone close to G1
        detection_active = False
        nav_active = "point"
        wp_average = None
        wp_input_history = []
        rospy.loginfo("pass G1 blind, no detection")
        publisher_state_auto.publish(state_auto + 1)                   # 13 - pass G1 blind, no detection

    elif state_auto == 13 and navigation_distance < 0.3:               # G1 passed
        wp_blind = None
        rospy.loginfo("fly blind towards G2, no detection")
        publisher_state_auto.publish(20)                               # 20 - fly blind towards G2, no detection

    elif state_auto == 20 and navigation_distance < 0.3:               # blind flight completed
        detection_active = True
        wp_blind = None
        rospy.loginfo("fly blind towards G2, with detection")
        publisher_state_auto.publish(state_auto + 1)                   # 21 - fly blind towards G2, with detection

    elif state_auto == 21 and wp_average is not None:                  # G2 detected
        nav_active = "through"
        wp_blind = None
        rospy.loginfo("fly visual to G2")
        publisher_state_auto.publish(state_auto + 1)                   # 22 - fly visual to G2

    elif state_auto == 22 and navigation_distance < 0.5:               # drone close to G2
        detection_active = False
        nav_active = "point"
        wp_average = None
        wp_input_history = []
        rospy.loginfo("pass G2 blind, no detection")
        publisher_state_auto.publish(state_auto + 1)                   # 23 - pass G2 blind, no detection

    elif state_auto == 23 and navigation_distance < 0.3:               # G2 passed
        nav_active = "off"
        wp_blind = None
        rospy.loginfo("land")
        publisher_state_auto.publish(90)                               # 90 - land

    elif state_auto == 70 and navigation_distance < 0.3:               # blind flight completed
        nav_active = "point"
        # wp
        rospy.loginfo("fly blind towards G7, with detection")
        publisher_state_auto.publish(state_auto + 1)                   # 71 - fly blind towards G7, with detection

    elif state_auto == 71 and wp_average is not None:                  # G7 detected
        nav_active = "point"
        rospy.loginfo("fly visual to front of G7")
        publisher_state_auto.publish(state_auto + 1)                   # 72 - fly visual to front of G7

    elif state_auto == 72 and navigation_distance < 0.15:               # hovering in front of G7
        detection_active = True
        publisher_dynamic_detection_on.publish(True)
        nav_active = "dynamic"
        rospy.loginfo("Starting Dynamic Gate")
        publisher_state_auto.publish(state_auto + 1)                   # 73 - dynamic navigation flies through gate

    elif state_auto == 73 and detection_dynamic_data.state == 5:       # drone has finished dynamic passage
        detection_active = False
        publisher_dynamic_detection_on.publish(False)
        nav_active = "point"
        # wp
        rospy.loginfo("Completed Dynamic Gate")
        publisher_state_auto.publish(90)

    elif state_auto == 90 and state_bebop == 4:                        # drone initiated landing
        rospy.loginfo("landing")
        publisher_state_auto.publish(state_auto + 1)                   # 91 - landing

    elif state_auto == 91 and state_bebop == 0:                        # drone finished landing
        rospy.loginfo("mission finished")
        publisher_state_auto.publish(state_auto + 1)                   # 92 - mission finished
    return

    # elif state_auto == 11 and navigation_distance < 0.2:  # blind point reached, can't see gate
    #     #  wp_blind = None
    #     #  wp_look = None
    #     #  nav_active = "off"
    #     rospy.loginfo("can't change to visual")
    #     #  publisher_state_auto.publish(state_auto + 2)
    #     pass


def callback_zed_odometry_changed():
    return


def callback_bebop_odometry_changed(data):
    global bebop_odometry
    bebop_odometry = data

    global auto_driving_msg
    global wp_select
    global wp_visual
    global wp_visual_old
    global wp_input_history
    global wp_blind
    global wp_blind_old
    global wp_look
    global nav_active

    # state_machine_advancement (if conditions are met: distances, states, ...)
    navigation_distance = calculate_distance()

    rospy.loginfo("navigation distance")
    rospy.loginfo(navigation_distance)

    state_machine_advancement(navigation_distance)

    if bebop_odometry is None:
        rospy.loginfo("No position")
        rospy.loginfo("publish empty driving msg")
        publisher_auto_drive.publish(Auto_Driving_Msg())
        return

    # calculate visual wp
    rospy.loginfo("calculate visual WP")
    calculate_visual_wp()

    # calculate blind wp
    rospy.loginfo("calculate blind WP")
    calculate_blind_wp()

    # select applicable waypoint
    select_waypoint()

    # ensure there is a waypoint
    if wp_select is None:
        rospy.loginfo("No waypoints")
        rospy.loginfo("publish empty driving msg")
        publisher_auto_drive.publish(Auto_Driving_Msg())
        return

    # navigate to wp_select
    if nav_active == "off":
        rospy.loginfo("Navigation turned off")
        rospy.loginfo("publish empty driving msg")
        publisher_auto_drive.publish(Auto_Driving_Msg())
        return
    elif nav_active == "point":
        auto_driving_msg = navigate_point()
    elif nav_active == "through":
        auto_driving_msg = navigate_through()
    elif nav_active == "dynamic":
        auto_driving_msg = navigate_dynamic()

    publisher_auto_drive.publish(auto_driving_msg)
    rospy.loginfo("publish real driving msg")
    rospy.loginfo([auto_driving_msg.x, auto_driving_msg.y, auto_driving_msg.z, auto_driving_msg.r])


if __name__ == '__main__':
    # Enable killing the script with Ctrl+C.
    signal.signal(signal.SIGINT, signal_handler)

    rospy.init_node('main_navigation', anonymous=True)

    # Variables
    bebop_odometry = None
    state_auto = -1
    state_bebop = None
    wp_average = None
    wp_visual = None
    wp_visual_old = None
    wp_input_history = []
    wp_blind_takeoff_time = None
    wp_blind = None
    wp_blind_old = None
    wp_look = None
    wp_select = None
    detection_active = False
    detection_dynamic_data = cr.DynamicData()
    detection_dynamic_input_history = np.array([[], []])
    nav_active = "off"
    nav_point_PID_x_pos = cr.PID2(.7, 0.1, 4.0)
    nav_point_PID_y_pos = cr.PID2(.7, 0.1, 4.0)
    nav_point_PID_x_vel = cr.PID2(0.8, 0, 0.0)
    nav_point_PID_y_vel = cr.PID2(0.8, 0, 0.0)
    nav_point_PID_z_vel = cr.PID2(1.0, 0, 0.0)  # PID
    nav_point_PID_r_vel = cr.PID2(0.5, 0, 1.0)  # PID
    nav_through_PID_y_pos = cr.PID2(.7, 0.1, 3.0)
    nav_through_PID_x_vel = cr.PID2(0.3, 0, 0.0)  # PID
    nav_through_PID_y_vel = cr.PID2(0.3, 0, 0.0)
    nav_through_PID_z_vel = cr.PID2(1.0, 0, 0.0)  # PID
    nav_through_PID_r_vel = cr.PID2(0.8, 0, 1.0)  # PID
    nav_limit_x = .1  # .25
    nav_limit_y = .1  # .4
    nav_limit_z = .2  # .75
    nav_limit_r = 1.0  # 1
    auto_driving_msg = Auto_Driving_Msg()

    # Publishers
    publisher_state_auto = rospy.Publisher("/auto/state_auto",     Int32,                queue_size=1, latch=True)
    publisher_auto_drive = rospy.Publisher("/auto/auto_drive",     Auto_Driving_Msg,     queue_size=1, latch=True)
    publisher_wp_average = rospy.Publisher("/auto/wp_average",     WP_Msg,               queue_size=1, latch=True)
    publisher_wp_visual = rospy.Publisher("/auto/wp_visual",       WP_Msg,               queue_size=1, latch=True)
    publisher_wp_blind = rospy.Publisher("/auto/wp_blind",         WP_Msg,               queue_size=1, latch=True)
    publisher_wp_look = rospy.Publisher("/auto/wp_look",           WP_Msg,               queue_size=1, latch=True)
    publisher_wp_current = rospy.Publisher("/auto/wp_current",     WP_Msg,               queue_size=1, latch=True)
    publisher_nav_log = rospy.Publisher("/auto/navigation_logger", String,               queue_size=1, latch=True)
    publisher_visual_log = rospy.Publisher("/auto/visual_logger",  String,               queue_size=1, latch=True)
    publisher_dev_log = rospy.Publisher("/auto/dev_logger",        String,               queue_size=1, latch=True)
    publisher_dynamic_detection_on = rospy.Publisher("/auto/dynamic_detection_on", Bool, queue_size=1, latch=True)
    publisher_gate_size = rospy.Publisher("/auto/gate_size",       Float32,              queue_size=1, latch=True)

    # Subscribers
    rospy.Subscriber("/auto/state_auto", Int32, callback_states_changed, "state_auto")
    rospy.Subscriber("/bebop/odom", Odometry, callback_bebop_odometry_changed)
    # rospy.Subscriber("/zed/odom", Odometry, callback_zed_odometry_changed)
    rospy.Subscriber("/auto/gate_detection_result", Gate_Detection_Msg, callback_visual_gate_detection_changed)
    rospy.Subscriber("/auto/gate_detection_result_dynamic", Float64MultiArray, callback_visual_gate_dynamic_changed)
    rospy.Subscriber("/bebop/states/ardrone3/PilotingState/FlyingStateChanged", Ardrone3PilotingStateFlyingStateChanged,
                     callback_states_changed, "state_bebop")

    # initializes startup by publishing state 0
    publisher_state_auto.publish(0)

    # Wait until connection between ground and air is established
    while state_auto is None or state_auto == -1:
        publisher_state_auto.publish(0)
        rospy.loginfo("waiting None")
        time.sleep(0.5)
    while state_auto == 0:
        rospy.loginfo("waiting 0")
        time.sleep(0.5)
    while state_auto == 1:
        publisher_state_auto.publish(state_auto + 1)
        rospy.loginfo("waiting 1")
        time.sleep(0.5)
    publisher_state_auto.publish(2)

    publisher_gate_size.publish(1.4)
    rospy.loginfo("Jetson communicating")

    rospy.spin()
