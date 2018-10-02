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
from std_msgs.msg import Int32, String, Float64MultiArray, Float32MultiArray, Bool, Float32, Empty
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

    measurement = input_data.data  # [time, angle]

    # if detection_dynamic_input_history.shape[1] > 0:
    #     measurement_diff = measurement[1] - detection_dynamic_input_history[1][-1]
    #     measurement_diff = abs(math.atan2(math.sin(measurement_diff), math.cos(measurement_diff)))
    # else:
    #     measurement_diff = math.pi
    #
    # if measurement_diff > 90*math.pi/180:
    #     rospy.loginfo("visual hand calculated")
    #
    #     if detection_dynamic_data.period is None:
    #         detection_dynamic_input_history = np.append(detection_dynamic_input_history,
    #                                                     [[measurement[0]], [measurement[1]]], axis=1)
    #         if detection_dynamic_input_history.shape[1] > 10:
    #             rospy.loginfo("enough measurements")
    #             detection_dynamic_input_history = np.delete(detection_dynamic_input_history, 0, axis=1)
    #             print "detection_dynamic_input_history"
    #             print detection_dynamic_input_history
    #             # calculate std deviation of list
    #             periods = cr.calculate_periods(detection_dynamic_input_history)
    #             print "periods"
    #             print periods
    #             detection_dynamic_data.std_dev = np.std(periods)
    #             # when std dev is low enough, provide period
    #             if detection_dynamic_data.std_dev < 0.2:
    #                 rospy.loginfo("measurements accepted")
    #                 detection_dynamic_data.period = np.mean(periods)
    #                 rospy.loginfo("std_deviation:")
    #                 rospy.loginfo(detection_dynamic_data.std_dev)
    #             else:
    #                 rospy.loginfo("standard deviation too high:")
    #                 rospy.loginfo(detection_dynamic_data.std_dev)
    #         else:
    #             rospy.loginfo("collecting measurements")
    #             rospy.loginfo(detection_dynamic_input_history.shape[1])
    #
    #     # else:
    #     #     # add to list only if gate position is close to where it's supposed to be
    #     #     t_diff = np.array(measurement[0]) - detection_dynamic_input_history[0][-1]
    #     #     angle_theory = t_diff / (2*math.pi*detection_dynamic_data.period) + detection_dynamic_input_history[0][-1]
    #     #     angle_theory = angle_theory % (2 * math.pi)
    #     #     diff = math.fabs(angle_theory - measurement[1])
    #     #     diff = min(diff, 2 * math.pi - diff)
    #     #     if diff < 40 * math.pi / 180:
    #     #         rospy.loginfo("use detected pointer")
    #     #         detection_dynamic_input_history = np.append(detection_dynamic_input_history,
    #     #                                                     [[measurement[0]], [measurement[1]]], axis=1)
    #     #         detection_dynamic_input_history = np.delete(detection_dynamic_input_history, 0, axis=1)
    #     #         print "detection_dynamic_input_history"
    #     #         print detection_dynamic_input_history
    #     #         periods = cr.calculate_periods(detection_dynamic_input_history)
    #     #         print "periods"
    #     #         print periods
    #     #         detection_dynamic_data.std_dev = np.std(periods)
    #     #         detection_dynamic_data.period = np.mean(periods)
    #     #         print "std_dev"
    #     #         print detection_dynamic_data.std_dev
    #     #         print "period"
    #     #         print detection_dynamic_data.period
    #     #
    #     #     else:
    #     #         rospy.loginfo("discard detected pointer")
    #
    # if True and detection_dynamic_data.period is not None and not detection_dynamic_data.triggered: # nav_active == "fast"
    #     # calculate current pointer position based on 5 last measurements
    #     t_delta = time.time() - detection_dynamic_input_history[0][-5:]
    #     a_delta = 2 * math.pi * t_delta / detection_dynamic_data.period
    #     angles = a_delta + detection_dynamic_input_history[1][-5:]
    #     theta_current = math.atan2(np.sum(np.sin(angles)), np.sum(np.cos(angles)))
    #     if theta_current < 0:
    #         theta_current = theta_current + 2 * math.pi
    #     detection_dynamic_data.theta = theta_current
    #     rospy.loginfo("detection_dynamic_data.theta")
    #     rospy.loginfo(theta_current)
    #
    #     # calculate trigger angle and difference
    #     theta_trigger = -2 * math.pi * detection_dynamic_data.time_taken_to_gate / detection_dynamic_data.period
    #     angle_diff = abs(theta_trigger - theta_current)
    #
    #     if angle_diff < 30*math.pi/180:  # abs(2*math.pi/(detection_dynamic_data.period*5)*.7):
    #         # execute throttle and turn off own navigation
    #         detection_dynamic_data.triggered = True
    #         emergency_shutdown("1")
    #         full_throttle_executer(1.5)
    #         global nav_active
    #         nav_active = "off"
    #     else:
    #         # wait for rotation
    #         pass
    #
    # log_string = str(
    #     0) + ", " + str(
    #     measurement[1]) + ", " + str(
    #     detection_dynamic_data.counter) + ', ' + str(
    #     detection_dynamic_input_history.shape[1]) + ', ' + str(
    #     detection_dynamic_data.std_dev or 0) + ", " + str(
    #     detection_dynamic_data.period or 0) + ', ' + str(
    #     angle_theory or 0) + ", " + str(
    #     diff or 0) + ", " + str(
    #     theta_current or 0) + ", " + str(
    #     theta_trigger or 0) + ", " + str(
    #     angle_diff or 0) + ", " + str(
    #     int(detection_dynamic_data.triggered)) + ", " + str(
    #     0) + ", " + str(
    #     0) + ", " + str(
    #     0) + ", " + str(
    #     0) + ", " + str(
    #     0) + ", " + str(
    #     0) + ", " + str(
    #     0) + ", " + str(
    #     0) + ", " + str(
    #     0) + ", " + str(
    #     0) + ", " + str(
    #     0) + ", " + str(
    #     0) + ", " + str(
    #     0) + ", " + str(
    #     0) + ", " + str(
    #     0) + ", " + str(
    #     0) + ", " + str(
    #     0) + ", " + str(
    #     0) + ", " + str(
    #     0) + ", " + str(
    #     0) + ", " + str(
    #     0) + ", " + str(
    #     0) + ", " + str(
    #     0) + ", " + str(
    #     0) + ", " + str(
    #     0) + ", " + str(
    #     0) + ", " + str(
    #     0) + ", " + str(
    #     time.time() - t_log) + ", " + str(
    #     0) + ", " + str(
    #     0) + ", " + str(
    #     0) + ", " + str(
    #     0)
    #
    # publisher_nav_log.publish(log_string)

    if nav_active == "fast":
        detection_dynamic_input_history = np.append(detection_dynamic_input_history, [[measurement[0], measurement[1]]],
                                                    axis=0)
        if detection_dynamic_input_history.shape[0] > 10 and not detection_dynamic_data.triggered:
            # detection_dynamic_input_history = np.delete(detection_dynamic_input_history, 0, axis=0)
            # Data must be array with rows having value [time,measurement]
            # data = np.loadtxt(open('theta.csv', 'rb'), delimiter=',', skiprows=1)
            data = detection_dynamic_input_history
            # Resolution determines how accurate the fourier transform is
            resolution = 0.001

            # Plots:
            # [fourier series plot in frequency domain, in complex plane, angle history plot, angle propagation plot]:
            plots = [False, False, False, False]
            # nplots = 0
            # if (plots[0]): nplots += 1
            # if (plots[1]): nplots += 1
            # if (plots[2] or plots[3]): nplots += 1
            # fig = plt.figure(figsize=(10, nplots * 10))
            # cur_ax = 0

            # Frequency returned in revolutions per second
            # Multiply by 2pi to convert to [rad/s]
            # Last two parameters are showing plot in frequency domain, and plot in imaginary plane.
            # [freq, offset, four_plot, imag_plot] = cr.extract_freq(data, resolution, plots[0], plots[1])
            [freq, offset] = cr.extract_freq(data, resolution, plots[0], plots[1])
            if np.sum(np.sign(np.diff(np.unwrap(np.transpose(detection_dynamic_input_history)[1][:])))) < 0:
                freq = -freq
            global detection_dynamic_freqs
            detection_dynamic_freqs = np.append(detection_dynamic_freqs,freq)

            if detection_dynamic_freqs.shape[0] > 5:
                dev = np.std(detection_dynamic_freqs)
            else:
                dev = 2

            if detection_dynamic_freqs.shape[0] > 20:
                detection_dynamic_freqs = np.delete(detection_dynamic_freqs, 0)

            theta_current = None
            cur_t = time.time()
            angle_diff = None
            theta_trigger = None
            exec_time = None
            a_dev = None

            if dev < 0.01 and abs(freq)>0.01:

                # How many seconds in the future after the last data point
                # t_future = 0

                # Determine the angle predicted at that time
                # Last two parameters are whether or not to plot history, and plot propagation
                # current_angle = cr.angle_in_t_seconds(data, freq, offset, t_future, plots[2], plots[3])
                # plt.show()

                detection_dynamic_data.period = 1.0/freq
                t_delta = cur_t - np.transpose(detection_dynamic_input_history)[0][-5:]
                a_delta = 2 * math.pi * t_delta / detection_dynamic_data.period
                angles = a_delta + np.transpose(detection_dynamic_input_history)[1][-5:]
                angles = np.unwrap(angles)
                a_dev = np.std(angles)
                if a_dev < 20*math.pi/180:
                    theta_current = math.atan2(np.sum(np.sin(angles)), np.sum(np.cos(angles)))
                    if theta_current < 0:
                        theta_current = theta_current + 2 * math.pi
                    detection_dynamic_data.theta = theta_current

                    # calculate trigger angle and difference
                    theta_trigger = -2 * math.pi * detection_dynamic_data.time_taken_to_gate / detection_dynamic_data.period
                    while not (0 <= theta_trigger <= 2 * math.pi):
                        if theta_trigger <= 0:
                            theta_trigger = theta_trigger + 2 * math.pi
                        else:
                            theta_trigger = theta_trigger - 2 * math.pi

                    angle_diff = (theta_trigger - theta_current)*np.sign(detection_dynamic_data.period)
                    while not (0 <= angle_diff <= 2 * math.pi):
                        if angle_diff <= 0:
                            angle_diff = angle_diff + 2 * math.pi
                        else:
                            angle_diff = angle_diff - 2 * math.pi

                    exec_time = cur_t + angle_diff / (2*math.pi / (abs(detection_dynamic_data.period)))

                    # execute throttle and turn off own navigation
                    log_string = str(
                        0) + ", " + str(
                        freq) + ", " + str(
                        offset) + ', ' + str(
                        measurement[0]) + ", " + str(
                        measurement[1]) + ", " + str(
                        dev) + ', ' + str(
                        theta_current or 0) + ', ' + str(
                        cur_t) + ", " + str(
                        theta_trigger or 0) + ', ' + str(
                        angle_diff or 0) + ", " + str(
                        exec_time or 0) + ", " + str(
                        1) + ", " + str(
                        a_dev or 0) + ", " + str(
                        0) + ", " + str(
                        0) + ", " + str(
                        0) + ", " + str(
                        0) + ", " + str(
                        0) + ", " + str(
                        0) + ", " + str(
                        0) + ", " + str(
                        0) + ", " + str(
                        0) + ", " + str(
                        0) + ", " + str(
                        0) + ", " + str(
                        0) + ", " + str(
                        0) + ", " + str(
                        0) + ", " + str(
                        0) + ", " + str(
                        0) + ", " + str(
                        0) + ", " + str(
                        0) + ", " + str(
                        0) + ", " + str(
                        0) + ", " + str(
                        0) + ", " + str(
                        0) + ", " + str(
                        0) + ", " + str(
                        0) + ", " + str(
                        0) + ", " + str(
                        0) + ", " + str(
                        time.time()) + ", " + str(
                        0) + ", " + str(
                        0) + ", " + str(
                        0) + ", " + str(
                        0)

                    publisher_nav_log.publish(log_string)

                    if detection_dynamic_data.triggered:
                        return
                    else:
                        detection_dynamic_data.triggered = True
                        while exec_time > time.time():
                            time.sleep(0.01)
                        full_throttle_executer(1.5)
                        global nav_active
                        nav_active = "off"
                        return

            log_string = str(
                0) + ", " + str(
                freq) + ", " + str(
                offset) + ', ' + str(
                measurement[0]) + ", " + str(
                measurement[1]) + ", " + str(
                dev) + ', ' + str(
                theta_current or 0) + ', ' + str(
                cur_t) + ", " + str(
                theta_trigger or 0) + ', ' + str(
                angle_diff or 0) + ", " + str(
                exec_time or 0) + ", " + str(
                0) + ", " + str(
                a_dev or 0) + ", " + str(
                0) + ", " + str(
                0) + ", " + str(
                0) + ", " + str(
                0) + ", " + str(
                0) + ", " + str(
                0) + ", " + str(
                0) + ", " + str(
                0) + ", " + str(
                0) + ", " + str(
                0) + ", " + str(
                0) + ", " + str(
                0) + ", " + str(
                0) + ", " + str(
                0) + ", " + str(
                0) + ", " + str(
                0) + ", " + str(
                0) + ", " + str(
                0) + ", " + str(
                0) + ", " + str(
                0) + ", " + str(
                0) + ", " + str(
                0) + ", " + str(
                0) + ", " + str(
                0) + ", " + str(
                0) + ", " + str(
                0) + ", " + str(
                time.time()) + ", " + str(
                0) + ", " + str(
                0) + ", " + str(
                0) + ", " + str(
                0)

            publisher_nav_log.publish(log_string)


def callback_visual_gate_detection_changed(data):
    global wp_average
    global wp_input_history

    if not detection_active:
        rospy.loginfo("detection not active")
        log_string = "0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0"
        publisher_visual_log.publish(log_string)
        return

    if data.tvec == ():
        rospy.loginfo("empty visual input")
        log_string = "0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0"
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
        if len(wp_input_history) > 15:
            rospy.loginfo("enough measurements")
            del wp_input_history[0]
            # calculate std deviation of list
            average = cr.find_average(wp_input_history)
            std_deviation = cr.find_std_dev_waypoints(average, wp_input_history)
            # when std dev is low enough, provide waypoint
            if std_deviation < 0.25:
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
            str(1.0) + ", " + \
            str(time.time()-t_log)
    else:
        # now, add to list only if gate position is close to average
        distance = np.linalg.norm(wp_current.pos - wp_average.pos)
        hdg_diff = abs(math.atan2(math.sin(wp_current.hdg - wp_average.hdg), math.cos(wp_current.hdg - wp_average.hdg)))
        if distance < 0.4 and hdg_diff < 15*math.pi/180:
            rospy.loginfo("use detected gate")
            wp_input_history.append(wp_current)
            del wp_input_history[0]
            wp_average = cr.find_average(wp_input_history)
            rospy.loginfo("wp_average")
            rospy.loginfo(wp_average)
        else:
            rospy.loginfo("discard detected gate")

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
            str(2.0) + ", " + \
            str(time.time()-t_log)

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
    global wp_blind

    wp_visual_old = wp_visual or wp_visual_old

    if state_auto == 72:
        rospy.loginfo("state 72, calculate hover position")

        # hover 2.8m behind dynamic gate at -0.5 height
        gate_pos = wp_average.pos
        gate_heading = wp_average.hdg
        hover_distance = -3.2
        hover_alt = -0.5
        extra_dist = np.array(
            [hover_distance * math.cos(gate_heading), hover_distance * math.sin(gate_heading), hover_alt])
        wp_visual = cr.WP(gate_pos + extra_dist, gate_heading)

        wp_look = wp_average

    elif state_auto == 63:
        rospy.loginfo("state 63, calculate 2nd jungle position")

        # set WP 1m ahead in gate direction
        gate_pos = wp_average.pos
        gate_heading = wp_average.hdg
        hover_distance = 1.0
        extra_dist = np.array(
            [hover_distance * math.cos(gate_heading), hover_distance * math.sin(gate_heading), 0])
        wp_visual = cr.WP(gate_pos + extra_dist, gate_heading)

        wp_look = wp_visual

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


def calculate_relative_wp(start, vector, heading):
    rot_mat = np.array(
        [[math.cos(heading), -math.sin(heading), 0], [math.sin(heading), math.cos(heading), 0], [0, 0, 1]])
    vector_global = np.matmul(rot_mat, vector)
    return cr.WP(start + vector_global, 0)


def calculate_blind_waypoint(fly, look):
    global wp_blind
    global wp_look
    global wp_blind_old
    wp_blind_old = wp_blind or wp_blind_old

    if wp_scale is not None:
        fly = wp_scale * fly
        look = wp_scale * look

    if wp_visual_old is None:
        # waypoint calculation based on own position and pose
        rospy.loginfo("wp_visual_old not avail")
        bebop_position = bebop_odometry.pose.pose.position
        bebop_orientation = bebop_odometry.pose.pose.orientation
        bebop_p = [bebop_position.x, bebop_position.y, bebop_position.z]
        bebop_q = [bebop_orientation.x, bebop_orientation.y, bebop_orientation.z, bebop_orientation.w]

        blind_position = fly  # front, left , up
        blind_position_global = cr.qv_mult(bebop_q, blind_position) + bebop_p
        blind_position_global = blind_position_global.tolist()

        look_position = look  # front, left, up
        look_position_global = cr.qv_mult(bebop_q, look_position) + bebop_p
        look_position_global = look_position_global.tolist()

        wp_blind = cr.WP(blind_position_global, 0)
        wp_look = cr.WP(look_position_global, 0)

    else:
        # waypoint calculation based on last gate
        rospy.loginfo("wp_visual_old avail")
        fly_start = wp_visual_old.pos
        fly_vector = fly
        fly_x_heading = wp_visual_old.hdg

        look_start = wp_visual_old.pos
        look_vector = look
        look_x_heading = wp_visual_old.hdg

        wp_blind = calculate_relative_wp(fly_start, fly_vector, fly_x_heading)
        wp_look = calculate_relative_wp(look_start, look_vector, look_x_heading)

    rospy.loginfo("state " + str(state_auto) + ": blind wp set")
    rospy.loginfo("wp_blind")
    rospy.loginfo(wp_blind)


def select_waypoint():

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
        d_theta = 2 * math.pi + d_theta
    else:
        pass

    y_pos_error = -dist * math.sin(d_theta)
    y_vel_des = nav_through_PID_y_pos.update(y_pos_error)

    x_pos_error = cr.min_value(dist * math.cos(d_theta), 0.1)
    if dist > 2:
        x_vel_des = x_pos_error*max(cr.limit_value(1-4*abs(d_theta)/math.pi, 1.0), 0)
    else:
        x_vel_des = x_pos_error*max(cr.limit_value(1-12*abs(d_theta)/math.pi, 1.0), -.25)

    z_error = diff_global[2]

    r_error = -(angle - pos_theta)
    if r_error > math.pi:
        r_error = -2 * math.pi + r_error
    elif r_error < -math.pi:
        r_error = 2 * math.pi + r_error

    y_vel_des_sum = sum(y_vel_des)
    if y_vel_des_sum > 0.1:
        y_vel_des_sum = (y_vel_des_sum - 0.1)/3 + 0.1
    elif y_vel_des_sum < -0.1:
        y_vel_des_sum = (y_vel_des_sum + 0.1)/3 - 0.1

    y_vel_error = y_vel_des_sum - velocity.y
    x_vel_error = cr.limit_value(x_vel_des, 0.6) - velocity.x

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
        time.time()-t_log) + ", " + str(
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
        r_error = 2 * math.pi + r_error

    x_pos_error = diff_global[0]
    y_pos_error = diff_global[1]

    x_vel_des = nav_point_PID_x_pos.update(x_pos_error)
    y_vel_des = nav_point_PID_y_pos.update(y_pos_error)

    x_vel_des_sum = sum(x_vel_des)
    if x_vel_des_sum > 0.1:
        x_vel_des_sum = (x_vel_des_sum - 0.5)/3 + 0.5
    elif x_vel_des_sum < -0.1:
        x_vel_des_sum = (x_vel_des_sum + 0.5)/3 - 0.5

    y_vel_des_sum = sum(y_vel_des)
    if y_vel_des_sum > 0.1:
        y_vel_des_sum = (y_vel_des_sum - 0.5)/3 + 0.5
    elif y_vel_des_sum < -0.1:
        y_vel_des_sum = (y_vel_des_sum + 0.5)/3 - 0.5

    x_vel_error = x_vel_des_sum - global_vel[0]
    y_vel_error = y_vel_des_sum - global_vel[1]

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
        time.time()-t_log) + ", " + str(
        0) + ", " + str(
        0) + ", " + str(
        0) + ", " + str(
        0)

    publisher_nav_log.publish(log_string)
    return msg
    # rospy.loginfo("calculated")
    # rospy.loginfo(auto_driving_msg)


def navigate_jungle():
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
        d_theta = 2 * math.pi + d_theta
    else:
        pass

    y_pos_error = -dist * math.sin(d_theta)
    y_vel_des = nav_through_PID_y_pos.update(y_pos_error)

    x_pos_error = cr.min_value(dist * math.cos(d_theta), 0.1)
    if dist > 2:
        x_vel_des = x_pos_error * max(cr.limit_value(1 - 18 * abs(d_theta) / math.pi, 1.0), 0)
        if abs(d_theta) < math.pi / 18:
            x_vel_des = x_vel_des * max(1 - abs(velocity.y / .07), 0)
    elif dist > 1.2:
        x_vel_des = x_pos_error * max(cr.limit_value(1 - 36 * abs(d_theta) / math.pi, 1.0), -.02)
        if abs(d_theta) < math.pi / 36 and dist > 1.5:
            x_vel_des = x_vel_des * max(1 - abs(velocity.y / .07), 0)
    else:
        x_vel_des = x_pos_error * max((.1 - abs(y_pos_error)) / .1, -.02)

    z_error = diff_global[2]

    r_error = -(angle - pos_theta)
    if r_error > math.pi:
        r_error = -2 * math.pi + r_error
    elif r_error < -math.pi:
        r_error = 2 * math.pi + r_error

    y_vel_des_sum = sum(y_vel_des)
    if y_vel_des_sum > 0.1:
        y_vel_des_sum = (y_vel_des_sum - 0.1)/3 + 0.1
    elif y_vel_des_sum < -0.1:
        y_vel_des_sum = (y_vel_des_sum + 0.1)/3 - 0.1

    y_vel_error = y_vel_des_sum - velocity.y
    x_vel_error = cr.limit_value(x_vel_des, 0.5) - velocity.x

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
        time.time()-t_log) + ", " + str(
        0) + ", " + str(
        0) + ", " + str(
        0) + ", " + str(
        0)

    publisher_nav_log.publish(log_string)
    return msg


def navigate_jungle2():
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

    diff_global_look = wp_look.pos - bebop_p
    pos_theta = math.atan2(diff_global_look[1], diff_global_look[0])
    angle = tfs.euler_from_quaternion(bebop_q)[2]

    r_error = -(angle - pos_theta)
    if r_error > math.pi:
        r_error = -2 * math.pi + r_error
    elif r_error < -math.pi:
        r_error = 2 * math.pi + r_error

    diff_global = wp_select.pos - bebop_p
    dist = math.hypot(diff_global[0], diff_global[1])

    commanding = .25

    x_pos_error = diff_global[0] * math.cos(-angle) - diff_global[1] * math.sin(-angle)
    y_pos_error = 3 * (diff_global[1] * math.cos(-angle) + diff_global[0] * math.sin(-angle))

    nav_cmd_x = (x_pos_error / math.hypot(x_pos_error, y_pos_error)) * commanding
    nav_cmd_y = (y_pos_error / math.hypot(x_pos_error, y_pos_error)) * commanding

    nav_cmd_r = cr.limit_value(r_error, nav_limit_r)

    msg = Auto_Driving_Msg()
    msg.x = nav_cmd_x
    msg.y = nav_cmd_y
    msg.z = 0
    msg.r = nav_cmd_r

    log_string = str(
         dist) + ", " + str(
        diff_global[0])+', ' + str(
        diff_global[1])+', ' + str(
        diff_global[2])+', ' + str(
        msg.x) + ", " + str(
        msg.y) + ", " + str(
        x_pos_error) + ", " + str(
        y_pos_error) + ", " + str(
        0) + ", " + str(
        0) + ", " + str(
        0) + ", " + str(
        0) + ", " + str(
        0) + ", " + str(
        0) + ", " + str(
        0) + ", " + str(
        0) + ", " + str(
        0) + ", " + str(
        0) + ", " + str(
        0) + ", " + str(
        0) + ", " + str(
        0) + ", " + str(
        0) + ", " + str(
        0) + ", " + str(
        0) + ", " + str(
        0) + ", " + str(
        0) + ", " + str(
        0) + ", " + str(
        0) + ", " + str(
        0) + ", " + str(
        0) + ", " + str(
        0) + ", " + str(
        0) + ", " + str(
        0) + ", " + str(
        0) + ", " + str(
        0) + ", " + str(
        0) + ", " + str(
        0) + ", " + str(
        0) + ", " + str(
        0) + ", " + str(
        time.time()-t_log) + ", " + str(
        0) + ", " + str(
        0) + ", " + str(
        0) + ", " + str(
        0)

    publisher_nav_log.publish(log_string)

    return msg


def full_throttle_executer(duration):
    msg_brake = Auto_Driving_Msg()
    msg_thrust = Auto_Driving_Msg()
    msg_thrust.x = 1.0

    time_end = time.time() + duration
    while time.time() < time_end:
        rospy.loginfo("DYN - full thrust command")
        publisher_auto_drive.publish(msg_thrust)
        time.sleep(0.01)

    while not state_bebop == cr.Bebop.HOVERING:
        rospy.loginfo("DYN - brake command")
        publisher_auto_drive.publish(msg_brake)
        time.sleep(0.1)


def navigate_dynamic():
    global detection_dynamic_data
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

    diff_global_look = wp_visual.pos - bebop_p
    angle = tfs.euler_from_quaternion(bebop_q)[2]
    pos_theta = math.atan2(diff_global_look[1], diff_global_look[0])
    r_error = -(angle - pos_theta)
    if r_error > math.pi:
        r_error = -2 * math.pi + r_error
    elif r_error < -math.pi:
        r_error = 2 * math.pi + r_error

    msg = Auto_Driving_Msg()

    if detection_dynamic_data.rotate_perform:
        # test and perform rotation
        if abs(r_error) < .08:
            detection_dynamic_data.rotate_perform = False
        else:
            msg.r = cr.limit_value(r_error, .1)
    elif detection_dynamic_data.timer is None:
        detection_dynamic_data.timer = time.time()
    elif time.time() - detection_dynamic_data.timer > .8:
        detection_dynamic_data.timer = None
        if abs(r_error) < .08:
            global nav_active
            nav_active = "fast"
        else:
            detection_dynamic_data.rotate_perform = True

        log_string = str(
            r_error) + ", " + str(
            0) + ", " + str(
            0) + ', ' + str(
            0) + ', ' + str(
            0) + ', ' + str(
            0) + ", " + str(
            0) + ", " + str(
            0) + ", " + str(
            0) + ", " + str(
            0) + ", " + str(
            0) + ", " + str(
            0) + ", " + str(
            0) + ", " + str(
            0) + ", " + str(
            0) + ", " + str(
            0) + ", " + str(
            0) + ", " + str(
            0) + ", " + str(
            0) + ", " + str(
            0) + ", " + str(
            0) + ", " + str(
            0) + ", " + str(
            0) + ", " + str(
            0) + ", " + str(
            0) + ", " + str(
            0) + ", " + str(
            0) + ", " + str(
            0) + ", " + str(
            0) + ", " + str(
            0) + ", " + str(
            0) + ", " + str(
            0) + ", " + str(
            0) + ", " + str(
            0) + ", " + str(
            0) + ", " + str(
            0) + ", " + str(
            0) + ", " + str(
            0) + ", " + str(
            0) + ", " + str(
            time.time() - t_log) + ", " + str(
            0) + ", " + str(
            0) + ", " + str(
            0) + ", " + str(
            0)

        publisher_nav_log.publish(log_string)

    return msg

#
#
# def navigate_dynamic():
#     global detection_dynamic_data
#     bebop_position = bebop_odometry.pose.pose.position
#     bebop_orientation = bebop_odometry.pose.pose.orientation
#
#     bebop_p = [bebop_position.x, bebop_position.y, bebop_position.z]
#     bebop_q = [bebop_orientation.x, bebop_orientation.y, bebop_orientation.z, bebop_orientation.w]
#
#     bebop_x_vec = cr.qv_mult(bebop_q, [1, 0, 0])
#     hdg = math.atan2(bebop_x_vec[1], bebop_x_vec[0])
#     rospy.loginfo("fly from")
#     rospy.loginfo([bebop_p[0], bebop_p[1], bebop_p[2], hdg])
#     rospy.loginfo("fly to")
#     rospy.loginfo(wp_select)
#
#     diff_global_look = wp_visual.pos - bebop_p
#     angle = tfs.euler_from_quaternion(bebop_q)[2]
#     pos_theta = math.atan2(diff_global_look[1], diff_global_look[0])
#     r_error = -(angle - pos_theta)
#     if r_error > math.pi:
#         r_error = -2 * math.pi + r_error
#     elif r_error < -math.pi:
#         r_error = 2 * math.pi + r_error
#
#     msg = Auto_Driving_Msg()
#
#     rospy.loginfo("DYN - dynamic state")
#     rospy.loginfo(detection_dynamic_data.state)
#
#     # stabilize
#     if detection_dynamic_data.state == 0:
#         # start a timer
#         if detection_dynamic_data.timer != 0.0:
#             # when timer has elapsed, check yaw error
#             if time.time()-detection_dynamic_data.timer > .8:
#                 rospy.loginfo("DYN - check yaw error")
#                 if abs(r_error) < .08:
#                     rospy.loginfo("DYN - yaw ok, state 2")
#                     detection_dynamic_data.state = 2  # error is small -> wait for gate rotatiom
#                 else:
#                     rospy.loginfo("DYN - yaw error high, state 1")
#                     detection_dynamic_data.state = 1  # error is large -> correct yaw
#                 detection_dynamic_data.timer = 0.0
#         else:
#             detection_dynamic_data.timer = time.time()
#             rospy.loginfo("DYN - timer started")
#
#     # rotate
#     elif detection_dynamic_data.state == 1:
#         msg = Auto_Driving_Msg()
#         # check error again before sending command
#         if abs(r_error) < .08:
#             rospy.loginfo("DYN - yaw ok, state 0")
#             detection_dynamic_data.state = 0
#         else:
#             rospy.loginfo("DYN - yawing")
#             msg.r = cr.limit_value(r_error, .1)
#
#     # wait for gate rotation
#     elif detection_dynamic_data.state == 2:
#         if detection_dynamic_data.period is None:
#             rospy.loginfo("DYN - no period yet, wait")
#         else:
#             angle_difference = abs(detection_dynamic_data.theta-detection_dynamic_data.theta_trigger())
#             if angle_difference < abs(2*math.pi/(detection_dynamic_data.period*5)*.7):
#                 rospy.loginfo("DYN - pointer angle triggered, state 3")
#                 detection_dynamic_data.state = 3
#                 # during execution of full throttle other instances of the callback go into mode 3 and wait there
#                 full_throttle_executer(1.3)
#                 detection_dynamic_data.state = 4
#                 # at this point a break message will be returned
#                 rospy.loginfo("DYN - second break command")
#             else:
#                 rospy.loginfo("DYN - wait for rotation")
#
#     # wait
#     elif detection_dynamic_data.state == 3:
#         # wait here and don't return a msg
#         while detection_dynamic_data.state == 3:
#             time.sleep(0.1)
#         rospy.loginfo("DYN - stacked break command")
#
#     # brake
#     elif detection_dynamic_data.state == 4:
#         if state_bebop == 2:
#             # advance own state to advance state machine
#             rospy.loginfo("DYN - completed")
#             detection_dynamic_data.state = 5
#         else:
#             # send brake command
#             rospy.loginfo("DYN - real break command")
#
#     try:
#         trig = detection_dynamic_data.theta_trigger()
#     except:
#         trig = 0
#
#     try:
#         diff = detection_dynamic_data.theta - trig
#     except:
#         diff = 0
#
#     log_string = str(
#         detection_dynamic_data.state or -1) + ", " + str(
#         detection_dynamic_data.period or 0) + ", " + str(
#         detection_dynamic_data.theta or 0)+', ' + str(
#         trig or 0)+', ' + str(
#         diff or 0) + ', ' + str(
#         r_error or 0) + ", " + str(
#         detection_dynamic_data.std_dev) + ", " + str(
#         0) + ", " + str(
#         0) + ", " + str(
#         0) + ", " + str(
#         0) + ", " + str(
#         0) + ", " + str(
#         0) + ", " + str(
#         0) + ", " + str(
#         0) + ", " + str(
#         0) + ", " + str(
#         0) + ", " + str(
#         0) + ", " + str(
#         0) + ", " + str(
#         0) + ", " + str(
#         0) + ", " + str(
#         0) + ", " + str(
#         0) + ", " + str(
#         0) + ", " + str(
#         0) + ", " + str(
#         0) + ", " + str(
#         0) + ", " + str(
#         0) + ", " + str(
#         0) + ", " + str(
#         0) + ", " + str(
#         0) + ", " + str(
#         0) + ", " + str(
#         0) + ", " + str(
#         0) + ", " + str(
#         0) + ", " + str(
#         0) + ", " + str(
#         0) + ", " + str(
#         0) + ", " + str(
#         0) + ", " + str(
#         time.time()-t_log) + ", " + str(
#         0) + ", " + str(
#         0) + ", " + str(
#         0) + ", " + str(
#         0)
#
#     publisher_nav_log.publish(log_string)
#
#     return msg


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
        angular_diff = abs(own_heading - pos_theta)
        angular_diff = min(angular_diff, 2*math.pi-angular_diff)
        return linear_distance + angular_diff * (180.0 / 15 * 0.3) / math.pi  # 15deg offset equal 30cm

    elif nav_active == "through" or nav_active == "jungle" or nav_active == "jungle2":
        flat_distance = np.linalg.norm([diff_global[0], diff_global[1], 0])
        heading_to_gate = math.atan2(wp_select.pos[1] - bebop_position.y, wp_select.pos[0] - bebop_position.x)
        heading_of_gate = wp_select.hdg
        heading_difference = heading_to_gate - heading_of_gate
        return flat_distance * math.cos(heading_difference)


def callback_zed_odometry_changed():
    return


def wp_move(wp, dist, hdg):
    vec = dist * np.array([math.cos(hdg), math.sin(hdg), 0])
    return cr.WP(wp.pos + vec, wp.hdg)


class State:
    def __init__(self, own_state=None, next_state=None, condition_type=None, condition_thres=None,
                 exit_clear_visual=None, detection_active_bool=None, special_detection=None, nav_active_str=None,
                 gate_size=None, gate_color=None, fly=None, look=None):
        self.own_state = own_state
        self.next_state = next_state
        self.condition_type = condition_type
        self.condition_thres = condition_thres
        self.exit_clear_visual = bool(exit_clear_visual)
        self.detection_active = bool(detection_active_bool)
        self.dynamic_on = (special_detection == "dynamic")
        self.jungle_on = (special_detection == "jungle")
        self.nav_active = nav_active_str
        self.gate_size = gate_size
        self.gate_color = np.array(gate_color)
        self.fly = np.array(fly)
        self.look = np.array(look)
        self.time = None

    def enter(self):
        # do things when state is selected
        print("enter state " + str(self.own_state))
        global current_state
        current_state = states[self.own_state]
        publisher_state_auto.publish(self.own_state)

        # reset [0, 0, 0, 0.5]
        nav_point_PID_x_pos.reset()
        nav_point_PID_y_pos.reset()
        nav_point_PID_x_vel.reset()
        nav_point_PID_y_vel.reset()
        nav_point_PID_z_vel.reset()
        nav_point_PID_r_vel.reset()
        nav_through_PID_y_pos.reset()
        nav_through_PID_x_vel.reset()
        nav_through_PID_y_vel.reset()
        nav_through_PID_z_vel.reset()
        nav_through_PID_r_vel.reset()

        if self.condition_type == "time":
            self.time = time.time()

        if self.gate_size is not None:
            publisher_gate_size.publish(self.gate_size)

        if self.gate_color.any():
            msg = Float32MultiArray()
            msg.data = self.gate_color
            publisher_gate_color.publish(msg)
        global nav_active
        nav_active = self.nav_active

        global detection_active
        detection_active = self.detection_active

        publisher_dynamic_detection_on.publish(self.dynamic_on)
        publisher_jungle_detection_on.publish(self.jungle_on)

        if self.jungle_on:
            global wp_input_history
            for idx, wp in enumerate(wp_input_history):
                wp_input_history[idx] = wp_move(wp, 0.7, wp_average.hdg)
            global wp_average
            wp_average = cr.find_average(wp_input_history)

        # calculate blind waypoints
        if self.fly.any():
            rospy.loginfo("state " + str(self.own_state) + ": set blind waypoint")
            calculate_blind_waypoint(self.fly, self.look)

    def exit(self):
        # do things when state is finished
        print("exit state " + str(self.own_state))
        if self.exit_clear_visual:
            global wp_average
            global wp_input_history
            wp_average = None
            wp_input_history = []
        states[self.next_state].enter()

    def check(self, navigation_distance):
        # rospy.loginfo("state machine sees: auto " + str(state_auto) + " and bebop " + str(state_bebop))
        rospy.loginfo("state " + str(state_auto))

        if self.condition_type == "dist":
            if navigation_distance < self.condition_thres * (wp_scale or 1.0):
                self.exit()

        elif self.condition_type == "wp":
            if wp_average is not None:
                self.exit()

        elif self.condition_type == "bebop":
            if state_bebop == self.condition_thres:
                self.exit()

        elif self.condition_type == "time":
            if time.time() > self.time + self.condition_thres:
                self.exit()

        elif self.condition_type == "nav":
            if nav_active == self.condition_thres:
                self.exit()


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
    global wp_takeoff
    global wp_scale
    global nav_active

    # calculate map scale factor
    if wp_takeoff is None:
        bebop_position = bebop_odometry.pose.pose.position
        wp_takeoff = [bebop_position.x, bebop_position.y, bebop_position.z]
    if wp_scale is None and wp_visual is None and wp_visual_old is not None:
        diff = wp_visual_old.pos - wp_takeoff
        wp_scale = math.sqrt(diff[0]*diff[0] + diff[1]*diff[1]) / 4.0
        rospy.loginfo("wp_scale")
        rospy.loginfo(wp_scale)

    # state_machine_advancement (if conditions are met: distances, states, ...)
    navigation_distance = calculate_distance()
    rospy.loginfo("navigation distance")
    rospy.loginfo(navigation_distance)
    current_state.check(navigation_distance)

    if bebop_odometry is None:
        rospy.loginfo("No position")
        rospy.loginfo("publish empty driving msg")
        publisher_auto_drive.publish(Auto_Driving_Msg())
        return

    # calculate visual wp
    rospy.loginfo("calculate visual WP")
    calculate_visual_wp()

    # calculate blind wp
    # rospy.loginfo("calculate blind WP")
    # calculate_blind_wp()

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
    elif nav_active == "jungle":
        auto_driving_msg = navigate_jungle()
    elif nav_active == "jungle2":
        auto_driving_msg = navigate_jungle2()
    elif nav_active == "fast":
        return

    publisher_auto_drive.publish(auto_driving_msg)
    rospy.loginfo("publish real driving msg")
    rospy.loginfo([auto_driving_msg.x, auto_driving_msg.y, auto_driving_msg.z, auto_driving_msg.r])


def emergency_shutdown(_):
    rospy.loginfo("emergency shutdown")
    rospy.signal_shutdown("emergency shutdown")


if __name__ == '__main__':
    # Enable killing the script with Ctrl+C.
    signal.signal(signal.SIGINT, signal_handler)

    rospy.init_node('main_navigation', anonymous=False)

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
    wp_takeoff = None
    wp_scale = 1.0
    detection_active = False
    detection_dynamic_data = cr.OpenloopData()
    detection_dynamic_input_history = np.empty((0, 2))
    detection_dynamic_freqs = np.array([])
    nav_active = "off"
    nav_point_PID_x_pos = cr.PID2(.5, 0.1, 1.0)
    nav_point_PID_y_pos = cr.PID2(.5, 0.1, 1.0)
    nav_point_PID_x_vel = cr.PID2(0.4, 0, 0.4)
    nav_point_PID_y_vel = cr.PID2(0.4, 0, 0.4)
    nav_point_PID_z_vel = cr.PID(1.0, 0, 0.0)
    nav_point_PID_r_vel = cr.PID(0.5, 0, 1.0)
    nav_through_PID_y_pos = cr.PID2(.7, 0.1, 3.0)
    nav_through_PID_x_vel = cr.PID(0.3, 0, 0.0)
    nav_through_PID_y_vel = cr.PID2(0.3, 0, 0.0)
    nav_through_PID_z_vel = cr.PID(1.0, 0, 0.0)
    nav_through_PID_r_vel = cr.PID(0.8, 0, 1.0)
    nav_limit_x = .1  # .25
    nav_limit_y = .2  # .4
    nav_limit_z = .5  # .75
    nav_limit_r = 1.0  # 1
    dist_gate_blind = 1.0  # how exact go to blind wp
    dist_gate_close = 0.5  # how soon turn off detection
    dist_exit_gate_wp = 15.0  # how far exit waypoint
    dist_egw = dist_exit_gate_wp
    dist_exit_gate_min = 0.5  # how far from the gate until it is cleared
    dist_gate_dyn = 0.2  # how accurate hover in front of dynamic gate
    dist_exit_jungle = dist_exit_gate_wp - (dist_exit_gate_min + 0.7)  # distance to exit jungle
    dist_exit_gate = dist_exit_gate_wp - dist_exit_gate_min  # distance to exit wp
    auto_driving_msg = Auto_Driving_Msg()
    current_state = None
    t_log = 1455208000

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
    publisher_jungle_detection_on = rospy.Publisher("/auto/jungle_detection_on", Bool,   queue_size=1, latch=True)
    publisher_gate_size = rospy.Publisher("/auto/gate_size",       Float32,              queue_size=1, latch=True)
    publisher_gate_color = rospy.Publisher("/auto/gate_color",     Float32MultiArray,    queue_size=1, latch=True)

    d = "dynamic"
    j = "jungle"
    j2 = "jungle2"
    p = "point"
    t = "through"
    o = "off"

    # own_state, next_state, condition_type, condition_thres, exit_clear_visual, detection_active_bool,
    # special_detection, nav_active_str, gate_size, gate_color, fly, look

    o0 = [87, 150, 50, 155, 255, 255]
    o1 = [105, 95, 40, 155, 255, 255]

    states = [State()] * 100
    states[02] = State(02, 03, "bebop", cr.Bebop.TAKEOFF,  0, 0, 0, o,  None, [], [], [])
    states[03] = State(03, 04, "bebop", cr.Bebop.HOVERING, 0, 0, 0, o,  None, [], [])
    states[04] = State(04, 10, "time",  1.0,               0, 0, 0, o,  None, o0, [], [])
    states[10] = State(10, 11, "dist",  dist_gate_blind,   0, 0, 0, p,  1.4,  o0, [0.0 0, 1.7], [3.2, 0, 0])
    states[11] = State(11, 12, "wp",    None,              0, 1, 0, p,  None, o0, [2.0, 0, 0], [3.2, 0, 0])
    states[12] = State(12, 13, "dist",  dist_gate_close,   1, 1, 0, t,  None, [], [], [])
    states[13] = State(13, 20, "dist",  dist_exit_gate,    0, 0, 0, p,  None, [], [dist_egw, 0, 0], [dist_egw, 0, 0])
    states[20] = State(20, 21, "dist",  dist_gate_blind,   0, 0, 0, p,  1.4,  [], [.7, 0, 0], [5.0, 0, 0])
    states[21] = State(21, 22, "wp",    None,              0, 1, 0, p,  None, [], [2.25, 0, 0], [5.0, 0, 0])
    states[22] = State(22, 23, "dist",  dist_gate_close,   1, 1, 0, t,  None, [], [], [])
    states[23] = State(23, 30, "dist",  dist_exit_gate,    0, 0, 0, p,  None, [], [dist_egw, 0, 0], [dist_egw, 0, 0])
    states[30] = State(30, 31, "dist",  dist_gate_blind,   0, 0, 0, p,  1.4,  [], [2.2, 0, 0], [2.75, -2.75, 0])
    states[31] = State(31, 32, "wp",    None,              0, 1, 0, p,  None, [], [2.75, -.25, 0], [2.75, -2.75, 0])
    states[32] = State(32, 33, "dist",  dist_gate_close,   1, 1, 0, t,  None, [], [], [])
    states[33] = State(33, 40, "dist",  dist_exit_gate,    0, 0, 0, p,  None, [], [dist_egw, 0, 0], [dist_egw, 0, 0])
    states[40] = State(40, 41, "dist",  dist_gate_blind,   0, 0, 0, p,  1.4,  [], [1.5, 0, 0.0], [2.18, -3.1, 0])
    states[41] = State(41, 42, "wp",    None,              0, 1, 0, p,  None, [], [2.2, -1, 0], [2.18, -3.1, 0])
    states[42] = State(42, 43, "dist",  dist_gate_close,   1, 1, 0, t,  None, [], [], [])
    states[43] = State(43, 50, "dist",  dist_exit_gate,    0, 0, 0, p,  None, [], [dist_egw, 0, 0], [dist_egw, 0, 0])
    states[50] = State(50, 51, "dist",  dist_gate_blind,   0, 0, 0, p,  1.4,  [], [1.5, 0, 0], [5.0, 0, 0])
    states[51] = State(51, 52, "wp",    None,              0, 1, 0, p,  None, [], [2.8, 0, 0], [5.0, 0, 0])
    states[52] = State(52, 53, "dist",  dist_gate_close,   1, 1, 0, t,  None, [], [], [])
    states[53] = State(53, 60, "dist",  dist_exit_gate,    0, 0, 0, p,  None, [], [dist_egw, 0, 0], [dist_egw, 0, 0])
    states[60] = State(60, 61, "dist",  dist_gate_blind,   0, 0, 0, p,  1.0,  [], [3.1, -2.7, -.1], [.5, -2.5, 0])
    states[61] = State(61, 62, "wp",    None,              0, 1, 0, p,  None, [], [2.9, -3.4, -1.0], [.5, -2.5, 0])
    states[62] = State(62, 63, "dist",  0.3,               0, 1, 0, j,  None, [], [], [])
    states[63] = State(63, 70, "dist",  0.9,               1, 1, j, j2, None, [], [], [])
    states[70] = State(70, 71, "dist",  dist_gate_blind,   0, 0, 0, p,  2.1,  o1, [0.5, -2.7, -0.2], [3.1, 0, 0])
    states[71] = State(71, 72, "wp",    None,              0, 1, 0, p,  None, o1, [2.9, -2.4, -0.2], [3.1, 0, 0])
    states[72] = State(72, 73, "dist",  dist_gate_dyn,     0, 1, 0, p,  None, o1, [], [])
    states[73] = State(73, 80, "nav",   "off",             1, 1, d, d,  None, [], [], [])
    states[80] = State(80, 81, "dist",  dist_gate_blind,   0, 0, 0, p,  1.4,  o0, [2.0, 0.0, 0.6], [2.5, 2.0, 0])
    states[81] = State(81, 82, "wp",    None,              0, 1, 0, p,  None, o0, [2.5, 0.0, 0.6], [2.5, 2.0, 0])
    states[82] = State(82, 83, "dist",  dist_gate_close,   1, 1, 0, t,  None, o0, [], [])
    states[83] = State(83, 90, "dist",  dist_exit_gate,    0, 0, 0, p,  None, [], [dist_egw, 0, 0], [dist_egw, 0, 0])
    states[90] = State(90, 91, "bebop", cr.Bebop.LANDING,  0, 0, 0, o,  None, [], [], [])
    states[91] = State(91, 91, "bebop", cr.Bebop.LANDED,   0, 0, 0, o,  None, [], [], [])

    # Subscribers
    rospy.Subscriber("/auto/state_auto", Int32, callback_states_changed, "state_auto")
    rospy.Subscriber("/bebop/odom", Odometry, callback_bebop_odometry_changed)
    # rospy.Subscriber("/zed/odom", Odometry, callback_zed_odometry_changed)
    rospy.Subscriber("/auto/gate_detection_result", Gate_Detection_Msg, callback_visual_gate_detection_changed)
    rospy.Subscriber("/auto/gate_detection_result_dynamic", Float64MultiArray, callback_visual_gate_dynamic_changed)
    rospy.Subscriber("/bebop/states/ardrone3/PilotingState/FlyingStateChanged", Ardrone3PilotingStateFlyingStateChanged,
                     callback_states_changed, "state_bebop")
    rospy.Subscriber("/auto/emergency_shutdown", Empty, emergency_shutdown)

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
    states[02].enter()

    rospy.loginfo("Jetson communicating")

    rospy.spin()
