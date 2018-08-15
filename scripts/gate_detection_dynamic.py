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

import signal
import sys


def signal_handler(signal, frame):
    sys.exit(0)


def mask_image(rgb, enc):
    # convert to HSV
    hsv = cv2.cvtColor(rgb, cv2.COLOR_BGR2HSV)

    # Threshold the HSV image to get only blue colors
    lower_color = np.array([100, 150, 100])  # blue 3D
    upper_color = np.array([140, 255, 255])  # blue 3D

    mask = cv2.inRange(hsv, lower_color, upper_color)

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
    global image_pub_dev1
    global bridge
    output_im = bridge.cv2_to_imgmsg(mask, encoding="8UC1")
    image_pub_dev1.publish(output_im)

    #show = cv2.resize(debug,(1280,720))
    #cv2.imshow("show",show)
    #if cv2.waitKey(1) & 0xFF == ord('q'):
    #   exit()


    global image_pub_threshold
    res = cv2.bitwise_and(rgb, rgb, mask=mask)
    output_im = bridge.cv2_to_imgmsg(res, encoding="rgb8")
    image_pub_threshold.publish(output_im)

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
    global detection_time_list
    global detection_angle_list
    global detection_velocity_list
    global detection_time_passthrough_list

    debug_on = True

    if debug_on:
        this_pose = Pose()
    else:
        if latest_pose is None:
            print("No position")
            return
        this_pose = latest_pose

    this_time = time.time()

    # convert image msg to matrix
    rgb = bridge.imgmsg_to_cv2(data, desired_encoding=data.encoding)

    # HSV conversion and frame detection
    mask = mask_image(rgb, data.encoding)

    # probabilistic hough transform
    minLineLength = 100
    maxLineGap = 10

    lines = cv2.HoughLinesP(mask, 5, np.pi / 180, 500, minLineLength=minLineLength, maxLineGap=maxLineGap)

    if lines is None:
        rospy.loginfo("no lines")
        result_publisher.publish(Gate_Detection_Msg())
        output_im = bridge.cv2_to_imgmsg(rgb, encoding=data.encoding)
        image_pub_gate.publish(output_im)
        return

    # lines have been found

    # calculate angles of all lines and create a border frame in the picture of the gate location
    angles = []
    distances = []
    indexes = list(reversed(range(len(lines))))

    gate = [745, 190]
    borders = [5000, 0, 5000, 0]
    for counter, line in enumerate(lines):
        for x1, y1, x2, y2 in line:
            angles.append(math.atan2(y2 - y1, x2 - x1) * 180 / np.pi)  # between -90 and 90
            distances.append((x1*(y2-y1)-y1*(x2-x1))/math.sqrt((x1-x2)**2+(y1-y2)**2))
            # cv2.line(rgb, (x1, y1), (x2, y2), (0, 0, 255), 2)
            borders[0] = min(borders[0], x1, x2)
            borders[1] = max(borders[1], x1, x2)
            borders[2] = min(borders[2], y1, y2)
            borders[3] = max(borders[3], y1, y2)

    # plt.clf()
    # #
    # ax = plt.axes()
    # ax.xaxis.set_major_locator(ticker.MultipleLocator(5))
    # ax.yaxis.set_minor_locator(ticker.MultipleLocator(100))
    # plt.plot(angles,distances,'bo')
    # plt.axis([-90, 90, -1500, 1500])
    # plt.grid(which='both')

    # average over all lines
    angle_m = angles[0]
    distance_m = distances[0]
    votes = indexes[0]
    i = 1
    while i < len(indexes):  # go through whole list once
        angle_1 = angles[i]
        distance_1 = distances[i]
        vote_1 = indexes[i]

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

    if angle_m >= 0:
        dist1 = math.sqrt((borders[0] - gate[0]) ** 2 + (borders[2] - gate[1]) ** 2)
        dist2 = math.sqrt((borders[1] - gate[0]) ** 2 + (borders[3] - gate[1]) ** 2)

        if dist1 > dist2:
            approx_angle = math.atan2(borders[2] - gate[1], borders[0] - gate[0])*180/math.pi
        else:
            approx_angle = math.atan2(borders[3] - gate[1], borders[1] - gate[0])*180/math.pi

        if abs(angle_m - approx_angle) > 90:
            angle_m = angle_m - 180
    else:
        dist1 = math.sqrt((borders[0] - gate[0]) ** 2 + (borders[3] - gate[1]) ** 2)
        dist2 = math.sqrt((borders[1] - gate[0]) ** 2 + (borders[2] - gate[1]) ** 2)

        if dist1 > dist2:
            approx_angle = math.atan2(borders[2] - gate[1], borders[0] - gate[0])*180/math.pi
        else:
            approx_angle = math.atan2(borders[3] - gate[1], borders[1] - gate[0])*180/math.pi

        if abs(angle_m - approx_angle) > 90:
            angle_m = angle_m + 180   # right is 0 deg

    detection_time_list = np.append(detection_time_list, this_time)
    detection_angle_list = np.append(detection_angle_list, angle_m)

    if len(detection_angle_list) > 20:

        detection_time_list = np.delete(detection_time_list, 0)
        detection_angle_list = np.delete(detection_angle_list, 0)

        sin_diff = math.sin((detection_angle_list[-1] - detection_angle_list[-2]) * math.pi / 180)
        cos_diff = math.cos((detection_angle_list[-1] - detection_angle_list[-2]) * math.pi / 180)
        angle_diff = math.atan2(sin_diff, cos_diff)*180/math.pi

        velocity = angle_diff/(detection_time_list[-1] - detection_time_list[-2])
        detection_velocity_list = np.append(detection_velocity_list, velocity)

        average_velocity = abs(np.mean(detection_velocity_list))

        diff_angle_list = detection_angle_list + 90 # difference to top position
        diff_angle_list[diff_angle_list<0] = diff_angle_list[diff_angle_list<0] + 360

        time_req_list = diff_angle_list / average_velocity
        #print time_req_list

        time_intersect_list = detection_time_list + time_req_list
        time_intersect_list[time_intersect_list < max(time_intersect_list)-0.5*(360/average_velocity)] = time_intersect_list[time_intersect_list < max(time_intersect_list)-0.5*(360/average_velocity)] + (360/average_velocity)
        time_intersect = np.mean(time_intersect_list)

    cv2.line(rgb, (gate[0], gate[1]), (gate[0]+int(250*math.cos(angle_m*math.pi/180)), gate[1]+int(250*math.sin(angle_m*math.pi/180))), (255,255,0),6)
    print 'run'


    output_im = bridge.cv2_to_imgmsg(rgb, encoding=data.encoding)
    image_pub_gate.publish(output_im)

    # plt.pause(0.01)
    # time.sleep(0.01)

def pose_callback(data):
    global latest_pose
    #latest_pose = data
    latest_pose = data.pose.pose


def camera_info_update(data):
    global camera_matrix
    camera_matrix = np.resize(np.asarray(data.K),[3,3])

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

    global image_pub_threshold
    global image_pub_gate
    global image_pub_dev1
    global image_pub_dev2
    global result_publisher
    global result_publisher_tvec
    global result_publisher_rvec
    global result_publisher_pose
    global latest_pose
    global detection_time_list
    global detection_angle_list
    global detection_velocity_list
    global detection_time_passthrough_list
    detection_time_list = np.array([])
    detection_angle_list = np.array([])
    detection_velocity_list = np.array([])
    detection_time_passthrough_list = np.array([])

    latest_pose = None

    rospy.Subscriber("/zed/rgb/image_rect_color", Image, stereo_callback)
    rospy.Subscriber("/bebop/odom", Odometry, pose_callback)

    image_pub_threshold = rospy.Publisher("/auto/gate_detection_threshold", Image, queue_size=1)
    image_pub_gate = rospy.Publisher("/auto/gate_detection_gate", Image, queue_size=1)
    image_pub_dev1 = rospy.Publisher("/auto/gate_detection_dev1", Image, queue_size=1)
    image_pub_dev2 = rospy.Publisher("/auto/gate_detection_dev2", Image, queue_size=1)

    result_publisher = rospy.Publisher("/auto/gate_detection_result", Gate_Detection_Msg, queue_size=1)
    result_publisher_tvec = rospy.Publisher("/auto/gate_detection_result_tvec", Float32MultiArray, queue_size=1)
    result_publisher_rvec = rospy.Publisher("/auto/gate_detection_result_rvec", Float32MultiArray, queue_size=1)
    result_publisher_pose = rospy.Publisher("/auto/gate_detection_result_pose", Pose, queue_size=1)

    global bridge
    bridge = CvBridge()

    rospy.loginfo("running")
    rospy.spin()

if __name__ == '__main__':
    main()
