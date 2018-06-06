#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError

import signal
import sys

def signal_handler(signal, frame):
    sys.exit(0)

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

        #convert to HSV
        hsv = cv2.cvtColor(rgb, cv2.COLOR_BGR2HSV)

        # Threshold the HSV image to get only orange colors
        lower_orange = np.array([1, 110, 80])
        upper_orange = np.array([11, 200, 200])
        mask = cv2.inRange(hsv, lower_orange, upper_orange)

        # Bitwise-AND mask and original image
        res = cv2.bitwise_and(rgb, rgb, mask=mask)

        #cv2.imshow('hsv', hsv)
        #cv2.imshow('mask', mask)
        cv2.imshow('res', res)
        cv2.waitKey(3)

        #img = self.bridge.cv2_to_imgmsg(res, "bgr8")


        #k = cv2.waitKey(5) & 0xFF
        #if k == 27:
        #    break


        #(rows,cols,channels) = cv_image.shape
        #if cols > 60 and rows > 60 :
        #    cv2.circle(cv_image, (50,50), 10, 255)
        #
        #cv2.imshow("Image window", cv_image)
        #cv2.waitKey(3)

        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(res, "bgr8"))
        except CvBridgeError as e:
            print(e)

        cameraMatrix = [396.17782, 0.0, 322.453185, 0.0, 399.798333, 174.243174, 0.0, 0.0, 1.0]
        vector<Point3f> objectPoints
        objectPoints.push_back(Point3f(0.44, 0.30, 0.46))
        #cv2.solvePnP(objectPoints, imagePoints, cameraMatrix, distCoeffs[, rvec[, tvec[, useExtrinsicGuess[, flags]]]]) ----> retval, rvec, tvec


if __name__ == '__main__':
    signal.signal(signal.SIGINT, signal_handler)
    rospy.init_node('bebop_image', anonymous=True)
    bebop_image()

    cameraMatrix = [396.17782, 0.0, 322.453185, 0.0, 399.798333, 174.243174, 0.0, 0.0, 1.0]
    vector<Point3f> objectPoints
    objectPoints.push_back(Point3f(0.44, 0.30, 0.46));
    #cv2.solvePnP(objectPoints, imagePoints, cameraMatrix, distCoeffs[, rvec[, tvec[, useExtrinsicGuess[, flags]]]]) ----> retval, rvec, tvec

    rospy.spin()
