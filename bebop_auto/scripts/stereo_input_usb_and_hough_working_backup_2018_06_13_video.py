#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
import math
import matplotlib.pyplot as plt
import time

import signal
import sys

camera_matrix = np.array(
        [[608.91474407072610, 0, 318.06264860718505],
         [0, 568.01764400596119, 242.18421070925399],
         [0, 0, 1]], dtype="double"
    )

def signal_handler(signal, frame):
    sys.exit(0)


def find_threshold_bimodal(array):
    array = np.array(array,dtype="double")
    array_sum = np.sum(array)
    var = []
    for n in range(array.size):
        partarray_sum = np.sum(array[:n])
        if partarray_sum == 0:
            s1 = 0
            var1 = 0
        else:
            s1 = np.sum(array[:n]) / array_sum
            mean1 = np.sum(array[:n] * range(n)) / partarray_sum
            var1 = np.sum(np.square(range(n) - mean1) * array[:n] / array_sum / s1)

        partarray_sum = np.sum(array[n:])
        if partarray_sum == 0:
            s2 = 0
            var2 = 0
        else:
            s2 = np.sum(array[n:]) / array_sum
            mean2 = np.sum(array[n:] * range(n, array.size)) / partarray_sum
            var2 = np.sum(np.square(range(n, array.size) - mean2) * array[n:] / array_sum / s2)
        var.append(int(s1 * var1 + s2 * var2))
    idx = (var.index(min(var)) + len(var) - 1 - var[::-1].index(min(var)))/2
    return idx


def isect_lines(line1, line2):
    for x1, y1, x2, y2 in line1:
        for x3, y3, x4, y4 in line2:
            s = float((x2 - x1) * (y3 - y1) - (x3 - x1) * (y2 - y1)) / float((x4 - x3) * (y2 - y1) - (x2 - x1) * (y4 - y3))
            x = x3 + s * (x4 - x3)
            y = y3 + s * (y4 - y3)
    return x, y


class bebop_image:
    def __init__(self):
        self.image_pub = rospy.Publisher("bebop_image_CV",Image)

        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/bebop/image_raw",Image,self.callback)

    def callback(self,data):
        try:
            rgb = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        # convert to HSV
        # hsv = cv2.cvtColor(rgb, cv2.COLOR_BGR2HSV)


def rectify_image(im):
    # Camera internals
    size = im.shape
    image_width = size[0]  # px
    sensor_width = 3.6  # mm
    focal_distance = 4.0  # mm
    focal_length = image_width * focal_distance / sensor_width

    # center = (size[1] / 2, size[0] / 2)
    # camera_matrix = np.array(
    #    [[focal_length, 0, center[0]],
    #     [0, focal_length, center[1]],
    #     [0, 0, 1]], dtype="double"
    # )

    #camera_matrix = np.array(
    #    [[608.91474407072610, 0, 318.06264860718505],
    #     [0, 568.01764400596119, 242.18421070925399],
    #     [0, 0, 1]], dtype="double"
    #)

    # print "Camera Matrix :\n {0}".format(camera_matrix)
    dist_coeffs = np.array(
        [[-4.37108312526e-01, 1.8776063621e-01, -4.6697911662e-03, 2.2242731991e-03 - 9.4929117169e-03]],
        dtype="double")

    # acquire its size
    h = size[0]
    w = size[1]

    # Generate new camera matrix from parameters
    newcameramatrix, roi = cv2.getOptimalNewCameraMatrix(camera_matrix, dist_coeffs, (w, h), 0)

    # Generate look-up tables for remapping the camera image
    mapx, mapy = cv2.initUndistortRectifyMap(camera_matrix, dist_coeffs, None, newcameramatrix, (w, h), 5)

    # Remap the original image to a new image
    im_rect = cv2.remap(im, mapx, mapy, cv2.INTER_LINEAR)

    return im_rect


def mask_image(im):
    # convert to HSV
    hsv = cv2.cvtColor(rgb, cv2.COLOR_BGR2HSV)

    # Threshold the HSV image to get only orange colors
    # lower_color = np.array([40, 0, 0])       # blue
    # upper_color = np.array([180, 150, 150])  # blue
    lower_color = np.array([6, 230, 110])  # orange
    upper_color = np.array([14, 255, 200])  # orange
    mask = cv2.inRange(hsv, lower_color, upper_color)

    # Bitwise-AND mask and original image
    # res = cv2.bitwise_and(rgb, rgb, mask=mask)
    # cv2.imshow('frame', res)
    # if cv2.waitKey(1) & 0xFF == ord('q'):
    #    break
    # continue

    return mask


if __name__ == '__main__':
    cap = cv2.VideoCapture(2)

    while True:
        # time.sleep(5)

        # Capture and rectify frames
        ret, rgb_in = cap.read()
        rgb = rectify_image(rgb_in)

        # Continue without lens distortion
        dist_coeffs = np.zeros((4, 1))

        # HSV conversion and frame detection
        mask = mask_image(rgb)

        # probabilistic hough transform
        minLineLength = 50
        maxLineGap = 30

        lines = cv2.HoughLinesP(mask, 1, np.pi / 180, 100, minLineLength, maxLineGap)
        if lines is not None:
            angles = []
            for counter, line in enumerate(lines):
                for x1, y1, x2, y2 in line:
                    angles.append(math.atan2(y2 - y1, x2 - x1)*180/np.pi)  # between -90 and 90

            hist, bin_edges = np.histogram(angles, 180, range=(-90, 90))
            idx = find_threshold_bimodal(hist)
            if idx >= 90:
                angle_thres = idx-90
            else:
                angle_thres = idx  # between 0 and 89

            linesh = []
            linesv = []
            disth = []
            distv = []

            for counter, line in enumerate(lines):
                for x1, y1, x2, y2 in line:
                    r = - float(x1 * (x2 - x1) + y1 * (y2 - y1)) / float((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1))
                    x = x1 + r*(x2-x1)
                    y = y1 + r*(y2-y1)
                    dist = math.sqrt(x*x+y*y)
                    cv2.circle(rgb, (int(x),int(y)), 10, (255, 255, 255), -1)

                    angle_difference = angle_thres - angles[counter]  # between -90 and 180
                    if 0 < angle_difference < 90:
                        linesh.append(line)
                        disth.append(dist)
                        #cv2.line(rgb, (x1, y1), (x2, y2), (255, 0, 0), 2)
                    else:
                        linesv.append(line)
                        distv.append(dist)
                        #cv2.line(rgb, (x1, y1), (x2, y2), (0, 0, 255), 2)

            hist_h, bin_edges = np.histogram(disth, 100, (0, 1000))
            idx_h = find_threshold_bimodal(hist_h)
            dist_th_h = idx_h * 10 + 5
            hist_v, bin_edges = np.histogram(distv, 100, (0, 1000))
            idx_v = find_threshold_bimodal(hist_v)
            dist_th_v = idx_v * 10 + 5

            linesh1 = []
            linesh2 = []
            linesv1 = []
            linesv2 = []

            for counter, line in enumerate(linesh):
                for x1, y1, x2, y2 in line:
                    if dist_th_h < disth[counter]:
                        linesh1.append(line)
                        cv2.line(rgb, (x1, y1), (x2, y2), (255, 255, 0), 2)
                    else:
                        linesh2.append(line)
                        cv2.line(rgb, (x1, y1), (x2, y2), (255, 0, 255), 2)

            for counter, line in enumerate(linesv):
                for x1, y1, x2, y2 in line:
                    if dist_th_v < distv[counter]:
                        linesv1.append(line)
                        cv2.line(rgb, (x1, y1), (x2, y2), (255, 255, 255), 2)
                    else:
                        linesv2.append(line)
                        cv2.line(rgb, (x1, y1), (x2, y2), (0, 255, 255), 2)

            c1 = len(linesv1) * len(linesh1)
            c2 = len(linesv1) * len(linesh2)
            c3 = len(linesv2) * len(linesh1)
            c4 = len(linesv2) * len(linesh2)
            isect1 = [0, 0]
            isect2 = [0, 0]
            isect3 = [0, 0]
            isect4 = [0, 0]
            for line1 in linesv1:
                for line2 in linesh1:
                    x,y = isect_lines(line1,line2)
                    isect1[0] = isect1[0] + x / c1
                    isect1[1] = isect1[1] + y / c1
                    # cv2.circle(rgb, (int(x), int(y)), 1, (255, 0, 0), -1)
                for line2 in linesh2:
                    x, y = isect_lines(line1, line2)
                    isect2[0] = isect2[0] + x / c2
                    isect2[1] = isect2[1] + y / c2
                    # cv2.circle(rgb, (x, y), 1, (0, 0, 0), -1)
            for line1 in linesv2:
                for line2 in linesh1:
                    x, y = isect_lines(line1, line2)
                    isect3[0] = isect3[0] + x / c3
                    isect3[1] = isect3[1] + y / c3
                    # cv2.circle(rgb, (x, y), 1, (0, 255, 0), -1)
                for line2 in linesh2:
                    x, y = isect_lines(line1, line2)
                    isect4[0] = isect4[0] + x / c4
                    isect4[1] = isect4[1] + y / c4
                    # cv2.circle(rgb, (x, y), 1, (0, 0, 255), -1)

            isect1 = (int(isect1[0]), int(isect1[1]))
            isect2 = (int(isect2[0]), int(isect2[1]))
            isect3 = (int(isect3[0]), int(isect3[1]))
            isect4 = (int(isect4[0]), int(isect4[1]))
            cv2.circle(rgb, isect1, 10, (255, 0, 0), -1)
            cv2.circle(rgb, isect2, 10, (0, 0, 0), -1)
            cv2.circle(rgb, isect3, 10, (0, 255, 0), -1)
            cv2.circle(rgb, isect4, 10, (0, 0, 255), -1)

            corner_points = np.array([isect1, isect2, isect3, isect4], dtype="double")

            # 3D model points.
            square_side = 0.1015
            model_points = np.array([
                (+square_side/2, +square_side/2, 0.0),
                (+square_side/2, -square_side/2, 0.0),
                (-square_side/2, +square_side/2, 0.0),
                (-square_side/2, -square_side/2, 0.0)])

            (success, rotation_vector, translation_vector) = cv2.solvePnP(model_points, corner_points, camera_matrix,
                                                                          dist_coeffs, flags=cv2.SOLVEPNP_ITERATIVE)

            # print "Rotation Vector:\n {0}".format(rotation_vector)
            # print "Translation Vector:\n {0}".format(translation_vector)

            # Project a 3D point (0, 0, 1000.0) onto the image plane.
            # We use this to draw a line sticking out of the plane

            (center_point_2D_base, jacobian_1) = cv2.projectPoints(np.array([(.0, .0, 0)]), rotation_vector,
                                                                   translation_vector, camera_matrix, dist_coeffs)
            (center_point_2D_back, jacobian_2) = cv2.projectPoints(np.array([(.0, .0, square_side)]), rotation_vector,
                                                                   translation_vector, camera_matrix, dist_coeffs)
            (center_point_2D_front, jacobian_2) = cv2.projectPoints(np.array([(.0, .0, -square_side)]), rotation_vector,
                                                                   translation_vector, camera_matrix, dist_coeffs)

            p1 = (int(center_point_2D_back[0][0][0]), int(center_point_2D_back[0][0][1]))
            p2 = (int(center_point_2D_front[0][0][0]), int(center_point_2D_front[0][0][1]))
            p3 = (int(center_point_2D_base[0][0][0]), int(center_point_2D_base[0][0][1]))

            if max(p1) < 10000 and max(p2) < 10000 and min(p1) > 0 and min(p2) > 0:
                cv2.line(rgb, p1, p2, (0, 0, 0), 10)
            if max(p3) < 10000 and min(p3) > 0:
                cv2.circle(rgb, p3, 10, (255, 255, 255), -1)


        # Display the resulting frame
        cv2.imshow('frame', rgb)
        # cv2.imshow('frame', hsv)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # When everything done, release the capture
    cap.release()
    cv2.destroyAllWindows()


#    signal.signal(signal.SIGINT, signal_handler)
#    rospy.init_node('bebop_image', anonymous=True)
#    bebop_image()

#    cameraMatrix = [396.17782, 0.0, 322.453185, 0.0, 399.798333, 174.243174, 0.0, 0.0, 1.0]
#    vector < Point3f > objectPoints
#    objectPoints.push_back(Point3f(0.44, 0.30, 0.46));
    # cv2.solvePnP(objectPoints, imagePoints, cameraMatrix, distCoeffs[, rvec[, tvec[, useExtrinsicGuess[, flags]]]]) ----> retval, rvec, tvec

#    rospy.spin()
