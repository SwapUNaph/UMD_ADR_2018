#!/usr/bin/env python

#  --- Changelog ---
# Goal:     Input by bebop camera and stereo camera and state machine and merged_odometry. Depending on state machine, use algorithms to detect gates on the image. Afterwards, position gates on a global map based on the odometry as gate orientation matrices are relative to camera orientation. Output position and orientation of next gate
# Status:   06/19:  Uses cheap stereo camera left video stream to identify gate position and orientation. Does not incorporate any odometry and publishes gate position at (1,0,0) with 3 Hz. Coordainte transformation missing. CV algorithm might need to be improved if there is a performance issue
#           07/06: Publishes gate position as seen from the camera (tvec: translation from camera, rvec: rotation from camera, and applicable bebop odometry

import rospy
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from sensor_msgs.msg import Image, CameraInfo
import cv2
import numpy as np
from cv_bridge import CvBridge
import time
import math
from bebop_auto.msg import Gate_Detection_Msg
from std_msgs.msg import Float32MultiArray
import matplotlib.pyplot as plt
import matplotlib.ticker as ticker

from sklearn.cluster import MeanShift, estimate_bandwidth

import signal
import sys


def signal_handler(signal, frame):
    sys.exit(0)

#
# def find_threshold_bimodal(array):
#     array = np.array(array,dtype="double")
#     array_sum = np.sum(array)
#     var = []
#     for n in range(array.size):
#         partarray_sum = np.sum(array[:n])
#         if partarray_sum == 0:
#             s1 = 0
#             var1 = 0
#         else:
#             s1 = np.sum(array[:n]) / array_sum
#             mean1 = np.sum(array[:n] * range(n)) / partarray_sum
#             var1 = np.sum(np.square(range(n) - mean1) * array[:n] / array_sum / s1)
#
#         partarray_sum = np.sum(array[n:])
#         if partarray_sum == 0:
#             s2 = 0
#             var2 = 0
#         else:
#             s2 = np.sum(array[n:]) / array_sum
#             mean2 = np.sum(array[n:] * range(n, array.size)) / partarray_sum
#             var2 = np.sum(np.square(range(n, array.size) - mean2) * array[n:] / array_sum / s2)
#         var.append(int(s1 * var1 + s2 * var2))
#     idx = (var.index(min(var)) + len(var) - 1 - var[::-1].index(min(var)))/2
#
#     if idx >= 90:
#         angle_thres = idx - 90
#     else:
#         angle_thres = idx
#
#     return angle_thres


def isect_lines(line1, line2):
    for x1, y1, x2, y2 in line1:
        for x3, y3, x4, y4 in line2:
            try:
                s = float((x2 - x1) * (y3 - y1) - (x3 - x1) * (y2 - y1)) / float(
                    (x4 - x3) * (y2 - y1) - (x2 - x1) * (y4 - y3))

                x = x3 + s * (x4 - x3)
                y = y3 + s * (y4 - y3)
                return x, y

            except ZeroDivisionError:
                rospy.loginfo("ZeroDivisionError in isect_lines")
                return -1, -1, -1


def mask_image_orange(rgb):
    # convert to HSV
    hsv = cv2.cvtColor(rgb, cv2.COLOR_BGR2HSV)

    # Threshold the HSV image to get only orange colors
    # lower_color = np.array([40, 0, 0])       # blue
    # upper_color = np.array([180, 150, 150])  # blue
    # lower_color = np.array([6, 230, 110])  # orange 2D
    # upper_color = np.array([14, 25, 200])  # orange 2D
    lower_color1 = np.array([0, 90, 80])  # orange 3D cypress
    upper_color1 = np.array([13, 255, 150])  # orange 3D cypress
    lower_color2 = np.array([170, 90, 80])  # orange 3D 150 cypress
    upper_color2 = np.array([180, 255, 150])  # orange 3D cypress

    mask1 = cv2.inRange(hsv, lower_color1, upper_color1)
    mask2 = cv2.inRange(hsv, lower_color2, upper_color2)
    mask = mask1+mask2

    # t0 = time.time()
    # try thinning all colored pixels from above (runs at around 5Hz)
    # thin = cv2.ximgproc.thinning(mask, mask, cv2.ximgproc.THINNING_ZHANGSUEN)

    # try a sobel edge detector (runs at around 20 Hz)
    # sobelx64f = cv2.Sobel(mask, cv2.CV_64F, 1, 0, ksize=3)
    # sobely64f = cv2.Sobel(mask, cv2.CV_64F, 0, 1, ksize=3)
    # abs_sobel64f = np.absolute(sobelx64f) + np.absolute(sobely64f)
    # sobel_8u = np.uint8(abs_sobel64f)

    # using a laplacian is very fast, around 100 Hz
    # laplacian = cv2.Laplacian(mask, cv2.CV_8U)

    # t1 = time.time()
    # total = t1 - t0
    # print(total)

    # Bitwise-AND mask and original image only for fun
    # global image_pub_dev1
    global bridge
    # output_im = bridge.cv2_to_imgmsg(mask, encoding="8UC1")
    # image_pub_dev1.publish(output_im)

    # show = cv2.resize(debug,(1280,720))
    # cv2.imshow("show",show)
    # if cv2.waitKey(1) & 0xFF == ord('q'):
    #    exit()

    global image_pub_threshold
    res = cv2.bitwise_and(rgb, rgb, mask=mask)
    output_im = bridge.cv2_to_imgmsg(res, encoding="rgb8")
    image_pub_threshold.publish(output_im)

    return mask


def mask_image_dynamic(rgb, enc):
    # convert to HSV
    hsv = cv2.cvtColor(rgb, cv2.COLOR_BGR2HSV)

    # Threshold the HSV image to get only blue colors
    lower_color = np.array([100, 150, 100])  # blue 3D
    upper_color = np.array([140, 255, 255])  # blue 3D

    mask = cv2.inRange(hsv, lower_color, upper_color)
    return mask


def find_angle(angle, a, b, c):
    # dist = a*sin(b*angle+c)
    return math.atan(a*b*math.cos(b*angle+c))


def stereo_callback(data):
    global valid_last_orientation
    global pose_pub
    global bridge
    global latest_pose
    global result_publisher
    global rvec
    global tvec
    global image_pub_gate

    debug_on = False

    if debug_on:
        this_pose = Pose()
    else:
        if latest_pose is None:
            print("No position")
            return
        this_pose = latest_pose
    this_time = time.time()

    # convert image msg to matrix
    input_image = bridge.imgmsg_to_cv2(data, desired_encoding=data.encoding)
    rgb = input_image

    # HSV conversion and frame detection
    mask = mask_image_orange(input_image)

    # probabilistic hough transform
    min_line_length = 100
    max_line_gap = 25

    lines = cv2.HoughLinesP(mask, 5, np.pi / 180, 500, minLineLength=min_line_length, maxLineGap=max_line_gap)

    if lines is None:
        rospy.loginfo("no lines")
        result_publisher.publish(Gate_Detection_Msg())
        output_im = bridge.cv2_to_imgmsg(rgb, encoding=data.encoding)
        image_pub_gate.publish(output_im)
        return

    # lines have been found
    # for counter, line in enumerate(lines):
    #     for x1, y1, x2, y2 in line:
    #         cv2.circle(rgb, (x1, y1), 5, (0, 255, 0), 2)
    #         cv2.circle(rgb, (x2, y2), 5, (255, 0, 0), 2)

    # cluster start and end points
    # for counter, line in enumerate(lines)
    dist_thresh = 50

    lines = np.array(lines)
    lines = np.squeeze(lines)
    corners = np.transpose(lines)
    start = corners[:2, :]
    end = corners[2:, :]
    start_end = np.concatenate((start, end), axis=1)
    # print start_end
    votes = list(reversed(range(len(lines))))
    votes = np.concatenate((votes, votes))
    x_ms_1 = np.array([])
    y_ms_1 = np.array([])
    vote_ms_1 = np.array([])

    # for i in range(np.shape(start)[1]):
    #     cv2.circle(rgb, (int(start[0][i]), int(start[1][i])), 5, (0, 0, 255), 2)

    # cluster starts
    clusters_1 = np.zeros(np.shape(start_end)[1])
    cluster = 0
    while not clusters_1.all():
        cluster = cluster+1
        s = np.where(clusters_1 == 0)[0][0]
        x_m = start_end[0][s]
        y_m = start_end[1][s]
        vote_m = votes[s]
        clusters_1[s] = cluster

        for idx in np.where(clusters_1 == 0)[0]:
            x = start_end[0][idx]
            y = start_end[1][idx]
            vote = votes[idx]
            distance = math.sqrt((x-x_m)**2+(y-y_m)**2)
            # print distance
            if distance < dist_thresh:
                x_m = (x_m * vote_m + x * vote) / (vote_m + vote)
                y_m = (y_m * vote_m + y * vote) / (vote_m + vote)
                vote_m = vote_m + vote
                clusters_1[idx] = cluster

        x_ms_1 = np.append(x_ms_1, x_m)
        y_ms_1 = np.append(y_ms_1, y_m)
        vote_ms_1 = np.append(vote_ms_1, vote_m)

    idx = vote_ms_1 * (1000-y_ms_1)

    max_cluster = np.argmax(idx)+1
    cv2.circle(rgb, (int(x_ms_1[max_cluster-1]), int(y_ms_1[max_cluster-1])), dist_thresh, (255, 0, 0), 2)
    corner_points = np.zeros((4, 2))
    corner_points[0, :] = [x_ms_1[max_cluster - 1], y_ms_1[max_cluster - 1]]

    # find lines originating from this cluster and recluster its endpoints
    to_cluster_id = np.where(clusters_1 == max_cluster)[0]
    to_cluster_id1 = to_cluster_id[to_cluster_id < len(lines)]
    to_cluster_id2 = to_cluster_id[to_cluster_id >= len(lines)] - len(lines)
    #
    # for line in to_cluster_id1:
    #     cv2.line(rgb, (start[0][line], start[1][line]), (end[0][line], end[1][line]), (0, 0, 255), 2)
    # for line in to_cluster_id2:
    #     cv2.line(rgb, (start[0][line], start[1][line]), (end[0][line], end[1][line]), (0, 0, 255), 2)

    to_cluster1 = end[:, to_cluster_id1]
    to_cluster2 = start[:, to_cluster_id2]
    to_cluster = np.concatenate((to_cluster1, to_cluster2), axis=1)
    votes_2 = np.concatenate((votes[to_cluster_id1], votes[to_cluster_id2]))

    clusters_2 = np.zeros(np.shape(to_cluster)[1])
    cluster = 0
    x_ms_2 = np.array([])
    y_ms_2 = np.array([])
    vote_ms_2 = np.array([])

    while not clusters_2.all():
        cluster = cluster+1
        s = np.where(clusters_2 == 0)[0][0]
        x_m = to_cluster[0][s]
        y_m = to_cluster[1][s]
        vote_m = votes_2[s]
        clusters_2[s] = cluster

        for idx in np.where(clusters_2 == 0)[0]:
            x = to_cluster[0][idx]
            y = to_cluster[1][idx]
            vote = votes_2[idx]
            distance = math.sqrt((x-x_m)**2+(y-y_m)**2)
            # print distance
            if distance < dist_thresh:
                x_m = (x_m * vote_m + x * vote) / (vote_m + vote)
                y_m = (y_m * vote_m + y * vote) / (vote_m + vote)
                vote_m = vote_m + vote
                clusters_2[idx] = cluster

        x_ms_2 = np.append(x_ms_2, x_m)
        y_ms_2 = np.append(y_ms_2, y_m)
        vote_ms_2 = np.append(vote_ms_2, vote_m)

    maximum_ids = vote_ms_2.argsort()[-2:][::-1]

    cv2.circle(rgb, (int(x_ms_2[maximum_ids[0]]), int(y_ms_2[maximum_ids[0]])), dist_thresh, (0, 255, 0), 2)
    cv2.circle(rgb, (int(x_ms_2[maximum_ids[1]]), int(y_ms_2[maximum_ids[1]])), dist_thresh, (0, 255, 0), 2)

    corner_points[1, :] = [x_ms_2[maximum_ids[0]], y_ms_2[maximum_ids[0]]]
    corner_points[2, :] = [x_ms_2[maximum_ids[1]], y_ms_2[maximum_ids[1]]]

    # find closest original cluster
    x_diff = x_ms_1 - x_ms_2[maximum_ids[0]]
    y_diff = y_ms_1 - y_ms_2[maximum_ids[0]]
    diff = x_diff * x_diff + y_diff * y_diff
    close_id1 = np.argmin(diff) + 1

    x_diff = x_ms_1 - x_ms_2[maximum_ids[1]]
    y_diff = y_ms_1 - y_ms_2[maximum_ids[1]]
    diff = x_diff * x_diff + y_diff * y_diff
    close_id2 = np.argmin(diff) + 1

    # find lines originating from two median points and recluster its endpoints
    to_cluster_id_b = np.concatenate((np.where(clusters_1 == close_id1)[0], np.where(clusters_1 == close_id2)[0]))
    #
    # t1 = np.where(clusters_1 == close_id1)[0]
    # t2 = np.where(clusters_1 == close_id2)[0]
    # to_cluster_id1_t1 = t1[t1 < len(lines)]
    # to_cluster_id2_t1 = t1[t1 >= len(lines)] - len(lines)
    # to_cluster_id1_t2 = t2[t2 < len(lines)]
    # to_cluster_id2_t2 = t2[t2 >= len(lines)] - len(lines)
    #
    # rm11 = np.in1d(to_cluster_id1_t1, to_cluster_id1) + np.in1d(to_cluster_id1_t1, to_cluster_id2)
    # rm21 = np.in1d(to_cluster_id2_t1, to_cluster_id1) + np.in1d(to_cluster_id2_t1, to_cluster_id2)
    # rm12 = np.in1d(to_cluster_id1_t2, to_cluster_id1) + np.in1d(to_cluster_id1_t2, to_cluster_id2)
    # rm22 = np.in1d(to_cluster_id2_t2, to_cluster_id1) + np.in1d(to_cluster_id2_t2, to_cluster_id2)
    #
    # tt11 = np.delete(to_cluster_id1_t1, np.where(rm11))
    # tt21 = np.delete(to_cluster_id2_t1, np.where(rm21))
    # tt12 = np.delete(to_cluster_id1_t2, np.where(rm12))
    # tt22 = np.delete(to_cluster_id2_t2, np.where(rm22))
    # print tt11, tt21
    # print tt12, tt22
    #
    # for line in tt11:
    #     cv2.line(rgb, (start[0][line], start[1][line]), (end[0][line], end[1][line]), (255, 255, 0), 2)
    # for line in tt21:
    #     cv2.line(rgb, (start[0][line], start[1][line]), (end[0][line], end[1][line]), (255, 255, 0), 2)
    # for line in tt12:
    #     cv2.line(rgb, (start[0][line], start[1][line]), (end[0][line], end[1][line]), (255, 0, 255), 2)
    # for line in tt22:
    #     cv2.line(rgb, (start[0][line], start[1][line]), (end[0][line], end[1][line]), (255, 0, 255), 2)

    to_cluster_id1_b = to_cluster_id_b[to_cluster_id_b < len(lines)]
    to_cluster_id2_b = to_cluster_id_b[to_cluster_id_b >= len(lines)] - len(lines)

    rm1 = np.in1d(to_cluster_id1_b, to_cluster_id1) + np.in1d(to_cluster_id1_b, to_cluster_id2)
    rm2 = np.in1d(to_cluster_id2_b, to_cluster_id1) + np.in1d(to_cluster_id2_b, to_cluster_id2)

    to_cluster_id1_b = np.delete(to_cluster_id1_b, np.where(rm1))
    to_cluster_id2_b = np.delete(to_cluster_id2_b, np.where(rm2))

    # for line in to_cluster_id1_b:
    #     cv2.line(rgb, (start[0][line], start[1][line]), (end[0][line], end[1][line]), (255, 255, 0), 2)
    # for line in to_cluster_id2_b:
    #     cv2.line(rgb, (start[0][line], start[1][line]), (end[0][line], end[1][line]), (255, 0, 255), 2)

    to_cluster1 = end[:, to_cluster_id1_b]
    to_cluster2 = start[:, to_cluster_id2_b]
    to_cluster = np.concatenate((to_cluster1, to_cluster2), axis=1)

    votes_3 = np.concatenate((votes[to_cluster_id1_b], votes[to_cluster_id2_b]))
    #
    # for i in range(np.shape(to_cluster)[1]):
    #     cv2.circle(rgb, (int(to_cluster[0][i]), int(to_cluster[1][i])), 2, (255, 0, 0), 2)

    clusters_3 = np.zeros(np.shape(to_cluster)[1])
    cluster = 0
    x_ms_3 = np.array([])
    y_ms_3 = np.array([])
    vote_ms_3 = np.array([])

    while not clusters_3.all():
        cluster = cluster + 1
        s = np.where(clusters_3 == 0)[0][0]
        x_m = to_cluster[0][s]
        y_m = to_cluster[1][s]
        vote_m = votes_3[s]
        clusters_3[s] = cluster

        for idx in np.where(clusters_3 == 0)[0]:
            x = to_cluster[0][idx]
            y = to_cluster[1][idx]
            vote = votes_3[idx]
            distance = math.sqrt((x - x_m) ** 2 + (y - y_m) ** 2)
            # print distance
            if distance < dist_thresh:
                x_m = (x_m * vote_m + x * vote) / (vote_m + vote)
                y_m = (y_m * vote_m + y * vote) / (vote_m + vote)
                vote_m = vote_m + vote
                clusters_3[idx] = cluster

        x_ms_3 = np.append(x_ms_3, x_m)
        y_ms_3 = np.append(y_ms_3, y_m)
        vote_ms_3 = np.append(vote_ms_3, vote_m)

    max_cluster = np.argmax(vote_ms_3)+1
    cv2.circle(rgb, (int(x_ms_3[max_cluster-1]), int(y_ms_3[max_cluster-1])), dist_thresh, (0, 0, 255), 2)

    corner_points[3, :] = [x_ms_3[max_cluster-1], y_ms_3[max_cluster-1]]
    print corner_points

    # Assume no lens distortion
    dist_coeffs = np.zeros((4, 1))

    # square_side = 1.03
    square_side = 1.4

    # 3D model points.
    model_points = np.array([
        (+square_side / 2, +square_side / 2, 0.0),
        (+square_side / 2, -square_side / 2, 0.0),
        (-square_side / 2, +square_side / 2, 0.0),
        (-square_side / 2, -square_side / 2, 0.0)])
    # t= time.time()
    (success, rvec, tvec) = cv2.solvePnP(model_points, corner_points, camera_matrix,
                                         dist_coeffs, flags=cv2.SOLVEPNP_ITERATIVE)

    rvec = np.squeeze(rvec)
    tvec = np.squeeze(tvec)
    # print time.time()-t
    # print "Rotation Vector:\n {0}".format(rvec)
    # print "Translation Vector:\n {0}".format(tvec)

    # publish results
    msg = Gate_Detection_Msg()
    msg.tvec = tvec
    msg.rvec = rvec
    msg.bebop_pose = this_pose
    result_publisher.publish(msg)
    rospy.loginfo("detected")

    # draw a line sticking out of the plane
    (center_point_2D_base, _) = cv2.projectPoints(np.array([(.0, .0, 0)]), rvec, tvec, camera_matrix, dist_coeffs)
    (center_point_2D_back, _) = cv2.projectPoints(np.array([(.0, .0, square_side)]), rvec, tvec, camera_matrix,
                                                  dist_coeffs)
    (center_point_2D_frnt, _) = cv2.projectPoints(np.array([(.0, .0, -square_side)]), rvec, tvec, camera_matrix,
                                                  dist_coeffs)
    p1 = (int(center_point_2D_back[0][0][0]), int(center_point_2D_back[0][0][1]))
    p2 = (int(center_point_2D_frnt[0][0][0]), int(center_point_2D_frnt[0][0][1]))
    p3 = (int(center_point_2D_base[0][0][0]), int(center_point_2D_base[0][0][1]))

    if True or max(p1) < 10000 and max(p2) < 10000 and min(p1) > 0 and min(p2) > 0:
        cv2.line(rgb, p1, p3, (0, 255, 255), 10)
        cv2.line(rgb, p2, p3, (0, 255, 255), 10)
    if True or max(p3) < 10000 and min(p3) > 0:
        cv2.circle(rgb, p3, 10, (0, 0, 0), -1)

    # Display the resulting frame
    # cv2.imshow('frame', rgb)

    if gate_detection_dynamic:
        global detection_time_list
        global detection_angle_list
        global detection_velocity_list
        global detection_time_passthrough_list
        global dynamic_publisher

        # HSV conversion and frame detection
        mask = mask_image_dynamic(input_image)

        # probabilistic hough transform
        minLineLength = 100
        maxLineGap = 10

        lines = cv2.HoughLinesP(mask, 5, np.pi / 180, 500, minLineLength=minLineLength, maxLineGap=maxLineGap)

        if lines is None:
            rospy.loginfo("no dynamic lines")
            result_publisher.publish(Gate_Detection_Msg())
            output_im = bridge.cv2_to_imgmsg(rgb, encoding=data.encoding)
            image_pub_gate.publish(output_im)
            return

        # lines have been found

        # calculate angles of all lines and create a border frame in the picture of the gate location
        angles = []
        distances = []
        indexes = list(reversed(range(len(lines))))

        gate = p3
        borders = [5000, 0, 5000, 0]
        for counter, line in enumerate(lines):
            for x1, y1, x2, y2 in line:
                angles.append(math.atan2(y2 - y1, x2 - x1) * 180 / np.pi)  # between -90 and 90
                distances.append((x1 * (y2 - y1) - y1 * (x2 - x1)) / math.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2))
                # cv2.line(rgb, (x1, y1), (x2, y2), (0, 0, 255), 2)
                borders[0] = min(borders[0], x1, x2)
                borders[1] = max(borders[1], x1, x2)
                borders[2] = min(borders[2], y1, y2)
                borders[3] = max(borders[3], y1, y2)

        # average over all lines
        angle_m = angles[0]
        distance_m = distances[0]
        votes = indexes[0]
        i = 1
        while i < len(indexes):  # go through whole list once
            angle_1 = angles[i]
            distance_1 = distances[i]
            vote_1 = indexes[i]

            # if angle difference to average is too large/small, change current angle accordingly
            a_dist = angle_1 - angle_m
            if a_dist > 90:
                if angle_1 < 0:
                    angle_1 = angle_1 + 180
                else:
                    angle_1 = angle_1 - 180
                distance_1 = - distance_1
            elif a_dist < -90:
                if angle_1 < 0:
                    angle_1 = angle_1 + 180
                else:
                    angle_1 = angle_1 - 180
                distance_1 = - distance_1

            angle_m = (angle_m * votes + angle_1 * vote_1) / (votes + vote_1)
            distance_m = (distance_m * votes + distance_1 * vote_1) / (votes + vote_1)
            votes = votes + vote_1

            i = i + 1

        # find approximate angle based on borders to identify quadrant
        if angle_m >= 0:
            dist1 = math.sqrt((borders[0] - gate[0]) ** 2 + (borders[2] - gate[1]) ** 2)
            dist2 = math.sqrt((borders[1] - gate[0]) ** 2 + (borders[3] - gate[1]) ** 2)

            if dist1 > dist2:
                approx_angle = math.atan2(borders[2] - gate[1], borders[0] - gate[0]) * 180 / math.pi
            else:
                approx_angle = math.atan2(borders[3] - gate[1], borders[1] - gate[0]) * 180 / math.pi

            if abs(angle_m - approx_angle) > 90:
                angle_m = angle_m - 180
        else:
            dist1 = math.sqrt((borders[0] - gate[0]) ** 2 + (borders[3] - gate[1]) ** 2)
            dist2 = math.sqrt((borders[1] - gate[0]) ** 2 + (borders[2] - gate[1]) ** 2)

            if dist1 > dist2:
                approx_angle = math.atan2(borders[2] - gate[1], borders[0] - gate[0]) * 180 / math.pi
            else:
                approx_angle = math.atan2(borders[3] - gate[1], borders[1] - gate[0]) * 180 / math.pi

            if abs(angle_m - approx_angle) > 90:
                angle_m = angle_m + 180  # right is 0 deg


        # publish finding
        rospy.loginfo("angle_m")
        rospy.loginfo(angle_m)
        dynamic_publisher.publish(np.array([this_time, angle_m]))

        # add finding to list
        # detection_time_list = np.append(detection_time_list, this_time)
        # detection_angle_list = np.append(detection_angle_list, angle_m)
        #
        # if len(detection_angle_list) > 20:
        #     detection_time_list = np.delete(detection_time_list, 0)
        #     detection_angle_list = np.delete(detection_angle_list, 0)
        #
        #     sin_diff = math.sin((detection_angle_list[-1] - detection_angle_list[-2]) * math.pi / 180)
        #     cos_diff = math.cos((detection_angle_list[-1] - detection_angle_list[-2]) * math.pi / 180)
        #     angle_diff = math.atan2(sin_diff, cos_diff) * 180 / math.pi
        #
        #     velocity = angle_diff / (detection_time_list[-1] - detection_time_list[-2])
        #     detection_velocity_list = np.append(detection_velocity_list, velocity)
        #
        #     average_velocity = abs(np.mean(detection_velocity_list))
        #
        #     diff_angle_list = detection_angle_list + 90  # difference to top position
        #     diff_angle_list[diff_angle_list < 0] = diff_angle_list[diff_angle_list < 0] + 360
        #
        #     time_req_list = diff_angle_list / average_velocity
        #     # print time_req_list
        #
        #     time_intersect_list = detection_time_list + time_req_list
        #     time_intersect_list[time_intersect_list < max(time_intersect_list) - 0.5 * (360 / average_velocity)] = \
        #     time_intersect_list[time_intersect_list < max(time_intersect_list) - 0.5 * (360 / average_velocity)] + (
        #                 360 / average_velocity)
        #     time_intersect = np.mean(time_intersect_list)
        #
        # cv2.line(rgb, (gate[0], gate[1]), (
        # gate[0] + int(250 * math.cos(angle_m * math.pi / 180)), gate[1] + int(250 * math.sin(angle_m * math.pi / 180))),
        #          (255, 255, 0), 6)
        # print 'run'

    output_im = bridge.cv2_to_imgmsg(rgb, encoding=data.encoding)
    image_pub_gate.publish(output_im)

    # plt.pause(0.001)
    # time.sleep(4)


def pose_callback(data):
    global latest_pose
    # latest_pose = data
    latest_pose = data.pose.pose


def camera_info_update(data):
    global camera_matrix
    camera_matrix = np.resize(np.asarray(data.K), [3, 3])

    # camera_matrix = np.array(
    #     [[608.91474407, 0, 318.06264861]
    #      [0, 568.01764401, 242.18421071]
    #     [0, 0, 1]], dtype="double"
    # )


def main():
    signal.signal(signal.SIGINT, signal_handler)
    rospy.init_node('gate_detection', anonymous=True)

    global camera_matrix
    camera_matrix = None
    rospy.Subscriber("/zed/rgb/camera_info", CameraInfo, camera_info_update)

    global valid_last_orientation
    valid_last_orientation = False

    global gate_detection_dynamic
    global image_pub_threshold
    global image_pub_gate
    global result_publisher
    global dynamic_publisher
    global latest_pose
    latest_pose = None
    gate_detection_dynamic = False

    global detection_time_list
    global detection_angle_list
    global detection_velocity_list
    global detection_time_passthrough_list
    detection_time_list = np.array([])
    detection_angle_list = np.array([])
    detection_velocity_list = np.array([])
    detection_time_passthrough_list = np.array([])

    rospy.Subscriber("/zed/rgb/image_rect_color", Image, stereo_callback)
    rospy.Subscriber("/bebop/odom", Odometry, pose_callback)

    image_pub_threshold = rospy.Publisher("/auto/gate_detection_threshold", Image, queue_size=1)
    image_pub_gate = rospy.Publisher("/auto/gate_detection_gate", Image, queue_size=1)
    result_publisher = rospy.Publisher("/auto/gate_detection_result", Gate_Detection_Msg, queue_size=1)
    dynamic_publisher = rospy.Publisher("/auto/gate_detection_result_dynamic", Float32MultiArray, queue_size=1)

    global bridge
    bridge = CvBridge()

    rospy.loginfo("running")
    rospy.spin()


if __name__ == '__main__':
    main()
