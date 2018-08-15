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
from std_msgs.msg import Int32, String, Float32MultiArray, Bool
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

def callback_visual_gate_dynamic_changed(data):
    global dynamic_detection_input_history
    print data.data
    # global dynamic_detection_velocity_list
    # global dynamic_detection_time_passthrough_list

    global dynamic_intersect_time

    if dynamic_intersect_time is None:
        dynamic_detection_input_history = np.append(dynamic_detection_input_history, data.data, axis=0)
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
            print("use detected gate")
            wp_input_history.append(wp_current)
            del wp_input_history[0]
            wp_average = cr.find_average(wp_input_history)
            rospy.loginfo("wp_average")
            rospy.loginfo(wp_average)
        else:
            print("discard detected gate")


    # add data to list
    detection_time_list = np.append(dynamic_detection_time_list, data.data[0])
    detection_angle_list = np.append(dynamic_detection_angle_list, data.data[1])

    if len(detection_angle_list) > 20:
        detection_time_list = np.delete(detection_time_list, 0)
        detection_angle_list = np.delete(detection_angle_list, 0)

        sin_diff = math.sin((detection_angle_list[-1] - detection_angle_list[-2]) * math.pi / 180)
        cos_diff = math.cos((detection_angle_list[-1] - detection_angle_list[-2]) * math.pi / 180)
        angle_diff = math.atan2(sin_diff, cos_diff) * 180 / math.pi

        velocity = angle_diff / (detection_time_list[-1] - detection_time_list[-2])
        detection_velocity_list = np.append(dynamic_detection_velocity_list, velocity)

        average_velocity = abs(np.mean(detection_velocity_list))

        diff_angle_list = detection_angle_list + 90  # difference to top position
        diff_angle_list[diff_angle_list < 0] = diff_angle_list[diff_angle_list < 0] + 360

        time_req_list = diff_angle_list / average_velocity
        # print time_req_list

        time_intersect_list = detection_time_list + time_req_list
        time_intersect_list[time_intersect_list < max(time_intersect_list) - 0.5 * (360 / average_velocity)] = \
        time_intersect_list[time_intersect_list < max(time_intersect_list) - 0.5 * (360 / average_velocity)] + (
                    360 / average_velocity)
        time_intersect = np.mean(time_intersect_list)


    print 'run'



def callback_visual_gate_detection_changed(data):
    global wp_average
    global wp_input_history

    if not detection_active:
        log_string = "0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0"
        visual_log_publisher.publish(log_string)
        return

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
    wp_current_publisher.publish(msg)

    rospy.loginfo("wp_current")
    rospy.loginfo(wp_current)

    # while there is no average yet: start collecting gate positions and evaluate standard deviation
    if wp_average is None:
        wp_input_history.append(wp_current)
        if len(wp_input_history) > 10:
            rospy.loginfo("enough measurements")
            del wp_input_history[0]
            # calculate std deviation of list
            average = cr.find_average(wp_input_history)
            std_deviation = cr.find_std_dev(average, wp_input_history)
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
            print("use detected gate")
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

    visual_log_publisher.publish(log_string)

    msg = WP_Msg()
    if wp_average is not None:
        msg.pos.x = wp_average.pos[0]
        msg.pos.y = wp_average.pos[1]
        msg.pos.z = wp_average.pos[2]
        msg.hdg = wp_average.hdg
    wp_average_publisher.publish(msg)

    return


def calculate_visual_wp():
    global wp_visual
    global wp_average
    global wp_visual_old

    if wp_visual is not None:
        wp_visual_old = wp_visual

    wp_visual = wp_average

    rospy.loginfo("wp_visual")
    rospy.loginfo(wp_visual)

    msg = WP_Msg()
    if wp_visual is not None:
        msg.pos.x = wp_visual.pos[0]
        msg.pos.y = wp_visual.pos[1]
        msg.pos.z = wp_visual.pos[2]
        msg.hdg = wp_average.hdg
    wp_visual_publisher.publish(msg)
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

    if wp_blind is not None:
        wp_blind_old = wp_blind

    if state_auto == 5 and wp_blind is None:
        rospy.loginfo("state 5: set fake start location to see first gate")

        bebop_p = [bebop_position.x, bebop_position.y, bebop_position.z]
        bebop_q = [bebop_orientation.x, bebop_orientation.y, bebop_orientation.z, bebop_orientation.w]

        blind_position = [0.0, 0.0, 0.9]  # front, left , up
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


def navigate_throu():
    bebop_position = bebop_odometry.pose.pose.position
    bebop_orientation = bebop_odometry.pose.pose.orientation
    # implementation exactly reverse as in matlab. Invert necessary when not required in matlab vice versa

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
    y_vel_des = nav_throu_PID_y_pos.update(y_pos_error)

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

    nav_cmd_x = nav_throu_PID_x_vel.update(x_vel_error)
    nav_cmd_y = nav_throu_PID_y_vel.update(y_vel_error)
    nav_cmd_z = nav_throu_PID_z_vel.update(z_error)
    nav_cmd_r = nav_throu_PID_r_vel.update(r_error)

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

    nav_log_publisher.publish(log_string)

    return msg
    # rospy.loginfo("calculated")
    # rospy.loginfo(auto_driving_msg)


def navigate_point():
    # implementation exactly reverse as in matlab. Invert necessary when not required in matlab vice versa
    bebop_q = [bebop_odometry.pose.pose.orientation.x, bebop_odometry.pose.pose.orientation.y,
               bebop_odometry.pose.pose.orientation.z, bebop_odometry.pose.pose.orientation.w]
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

    diff_global = wp_select.pos - [bebop_odometry.pose.pose.position.x, bebop_odometry.pose.pose.position.y,
                                   bebop_odometry.pose.pose.position.z]
    diff_global_look = wp_look.pos - [bebop_odometry.pose.pose.position.x, bebop_odometry.pose.pose.position.y,
                                      bebop_odometry.pose.pose.position.z]

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
    nav_log_publisher.publish(log_string)

    return msg
    # rospy.loginfo("calculated")
    # rospy.loginfo(auto_driving_msg)


def calculate_distance():
    if wp_select is None or bebop_odometry is None:
        return 999

    bebop_position = bebop_odometry.pose.pose.position
    diff_global = wp_select.pos - [bebop_position.x, bebop_position.y, bebop_position.z]
    if navigation_active == "point":
        return np.linalg.norm(diff_global)
    elif navigation_active == "throu":
        flat_distance = np.linalg.norm([diff_global[0], diff_global[1], 0])
        heading_to_gate = math.atan2(wp_select.pos[1] - bebop_position.y, wp_select.pos[0] - bebop_position.x)
        heading_of_gate = wp_select.hdg
        heading_difference = heading_to_gate - heading_of_gate
        return flat_distance * math.cos(heading_difference)


def state_machine_advancement(navigation_distance):
    global wp_blind
    global wp_blind_old
    global wp_look
    global navigation_active
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
    if state_auto == 2 and state_bebop == 1:  # drone is taking off
        rospy.loginfo("takeoff started")
        state_auto_publisher.publish(state_auto + 1)
    elif state_auto == 3 and state_bebop == 2:  # drone was taking off and is now hovering/flying
        rospy.loginfo("takeoff completed")
        wp_blind_takeoff_time = time.time()
        state_auto_publisher.publish(state_auto + 1)
    elif state_auto == 4 and wp_blind_takeoff_time + 3 < time.time():
        rospy.loginfo("waited 3s. start blind and detection")
        detection_active = True
        navigation_active = "point"
        state_auto_publisher.publish(state_auto + 1)
    elif state_auto == 5 and wp_average is not None:  # we detected the gate and try to get there
        navigation_active = "throu"
        wp_blind = None
        wp_look = None
        rospy.loginfo("visually detected gate")
        state_auto_publisher.publish(state_auto + 1)
    elif state_auto == 5 and navigation_distance < 0.2:  # we reached the blind point and can't see the gate
        #  wp_blind = None
        #  wp_look = None
        #  navigation_active = "off"
        #  rospy.loginfo("can't change to visual. Landing")
        #  state_auto_publisher.publish(state_auto + 2)
        pass
    elif state_auto == 6 and navigation_distance < 0.5:  # drone gets close to gate. continue in gate direction to turn
        detection_active = False
        navigation_active = "point"
        wp_blind_old = wp_blind
        wp_blind = None
        wp_look = None
        rospy.loginfo("gate almost reached")
        state_auto_publisher.publish(state_auto + 1)
    elif state_auto == 7 and navigation_distance < 0.2:  # drone is at turn. continue in gate direction to ldg
        detection_active = True
        navigation_active = "point"
        wp_blind_old = wp_blind
        wp_blind = None
        rospy.loginfo("halfway turned")
        state_auto_publisher.publish(state_auto + 1)
    elif state_auto == 8 and wp_average is not None:  # we detected the gate 2nd and try to get there
        navigation_active = "throu"
        wp_blind = None
        rospy.loginfo("visually detected gate 2nd")
        state_auto_publisher.publish(state_auto + 1)
    elif state_auto == 9 and navigation_distance < 0.5:  # drone gets close to gate. continue in gate direction to ldg
        detection_active = False
        navigation_active = "point"
        wp_blind = None
        rospy.loginfo("gate almost reached 2nd")
        state_auto_publisher.publish(state_auto + 1)
    elif state_auto == 10 and navigation_distance < 0.2:  # drone reached landing location
        wp_blind = None
        wp_look = None
        navigation_active = "off"
        rospy.loginfo("mission finished")
        state_auto_publisher.publish(98)
    elif state_auto == 98 and state_bebop == 4:  # drone initiated landing
        rospy.loginfo("landing started")
        state_auto_publisher.publish(state_auto + 1)
    elif state_auto == 99 and state_bebop == 0:  # drone was landing and has landed
        rospy.loginfo("landing completed")
        state_auto_publisher.publish(state_auto + 1)
    return


def callback_zed_odometry_changed():
    return


def callback_bebop_odometry_changed(data):
    global bebop_odometry
    bebop_odometry = data

    global auto_driving_msg
    global wp_select
    global min_distance
    global wp_visual
    global wp_visual_old
    global wp_input_history
    global wp_blind
    global wp_blind_old
    global wp_look
    global navigation_active

    # state_machine_advancement (if conditions are met: distances, states, ...)
    navigation_distance = calculate_distance()

    rospy.loginfo("navigation distance")
    rospy.loginfo(navigation_distance)

    state_machine_advancement(navigation_distance)

    if bebop_odometry is None:
        rospy.loginfo("No position")
        rospy.loginfo("publish empty driving msg")
        auto_drive_publisher.publish(Auto_Driving_Msg())
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
        auto_drive_publisher.publish(Auto_Driving_Msg())
        return

    # navigate to wp_select
    if navigation_active == "off":
        rospy.loginfo("Navigation turned off")
        rospy.loginfo("publish empty driving msg")
        auto_drive_publisher.publish(Auto_Driving_Msg())
        return
    elif navigation_active == "point":
        auto_driving_msg = navigate_point()
    elif navigation_active == "throu":
        auto_driving_msg = navigate_throu()

    auto_drive_publisher.publish(auto_driving_msg)
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
    gate_global_p_old = np.array([0,0,0])
    wp_average = None
    wp_visual = None
    wp_visual_old = None
    wp_input_history = []
    wp_blind_takeoff_time = None
    wp_blind = None
    wp_blind_old = None
    wp_look = None
    wp_select = None
    navigation_active = "off"
    detection_active = False
    nav_point_PID_x_pos = cr.PID2(.7, 0.1, 4.0)
    nav_point_PID_y_pos = cr.PID2(.7, 0.1, 4.0)
    nav_point_PID_x_vel = cr.PID2(0.8, 0, 0.0)
    nav_point_PID_y_vel = cr.PID2(0.8, 0, 0.0)
    nav_point_PID_z_vel = cr.PID(1.0, 0, 0.0)
    nav_point_PID_r_vel = cr.PID(0.5, 0, 1.0)
    nav_throu_PID_y_pos = cr.PID2(.7, 0.1, 3.0)
    nav_throu_PID_x_vel = cr.PID(0.3, 0, 0.0)
    nav_throu_PID_y_vel = cr.PID2(0.3, 0, 0.0)
    nav_throu_PID_z_vel = cr.PID(1.0, 0, 0.0)
    nav_throu_PID_r_vel = cr.PID(0.8, 0, 1.0)
    nav_limit_x = .1  # .25
    nav_limit_y = .1  # .4
    nav_limit_z = .2  # .75
    nav_limit_r = 1.0  # 1
    empty_command = True
    auto_driving_msg = Auto_Driving_Msg()
    min_distance = 999
    dynamic_detection_input_history = np.array([[], []])
    dynamic_detection_velocity_list = np.array([])
    dynamic_detection_time_passthrough_list = np.array([])
    dynamic_intersect_time = None

    # Publishers
    state_auto_publisher = rospy.Publisher("/auto/state_auto",     Int32,               queue_size=1, latch=True)
    auto_drive_publisher = rospy.Publisher("/auto/auto_drive",     Auto_Driving_Msg,    queue_size=1, latch=True)
    wp_average_publisher = rospy.Publisher("/auto/wp_average",     WP_Msg,              queue_size=1, latch=True)
    wp_visual_publisher = rospy.Publisher("/auto/wp_visual",       WP_Msg,              queue_size=1, latch=True)
    wp_blind_publisher = rospy.Publisher("/auto/wp_blind",         WP_Msg,              queue_size=1, latch=True)
    wp_look_publisher = rospy.Publisher("/auto/wp_look",           WP_Msg,              queue_size=1, latch=True)
    wp_current_publisher = rospy.Publisher("/auto/wp_current",     WP_Msg,              queue_size=1, latch=True)
    nav_log_publisher = rospy.Publisher("/auto/navigation_logger", String,              queue_size=1, latch=True)
    visual_log_publisher = rospy.Publisher("/auto/visual_logger",  String,              queue_size=1, latch=True)
    dev_log_publisher = rospy.Publisher("/auto/dev_logger",        String,              queue_size=1, latch=True)
    dynamic_detection_on_publisher = rospy.Publisher("/auto/dynamic_detection_on", Bool,queue_size=1, latch=True)

    # Subscribers
    rospy.Subscriber("/auto/state_auto", Int32, callback_states_changed, "state_auto")
    rospy.Subscriber("/bebop/odom", Odometry, callback_bebop_odometry_changed)
    # rospy.Subscriber("/zed/odom", Odometry, callback_zed_odometry_changed)
    rospy.Subscriber("/auto/gate_detection_result", Gate_Detection_Msg, callback_visual_gate_detection_changed)
    rospy.Subscriber("/auto/gate_detection_result_dynamic", Float32MultiArray, callback_visual_gate_dynamic_changed)
    rospy.Subscriber("/bebop/states/ardrone3/PilotingState/FlyingStateChanged", Ardrone3PilotingStateFlyingStateChanged,
                     callback_states_changed, "state_bebop")


    # initializes startup by publishing state 0
    state_auto_publisher.publish(0)

    # Wait until connection between ground and air is established
    while state_auto is None or state_auto == -1:
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
