#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
import cv2
import numpy as np
from cv_bridge import CvBridge
import signal
import sys
import os

def signal_handler(signal, frame):
    sys.exit(0)


def stereo_callback(data):
    # convert image msg to matrix
    rgb = bridge.imgmsg_to_cv2(data, desired_encoding=data.encoding)

    # convert image to HSV
    # original image is BGR but conversion here is RGB so red color does not wrap around
    hsv = cv2.cvtColor(rgb, cv2.COLOR_RGB2HSV)

    # masking image
    path = os.path.dirname(sys.argv[0]) + "/tune_colors_values.txt"
    f = open(path)
    low = f.readline().split(", ")
    high = f.readline().split(", ")
    lower_color = np.array(low, dtype='int')
    upper_color = np.array(high, dtype='int')

    mask = cv2.inRange(hsv, lower_color, upper_color)

    rgb = cv2.cvtColor(hsv, cv2.COLOR_HSV2RGB)
    res = cv2.bitwise_and(rgb, rgb, mask=mask)
    cv2.imshow("pic", res)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        exit()


if __name__ == '__main__':
    signal.signal(signal.SIGINT, signal_handler)
    rospy.init_node('tune_colors', anonymous=True)

    bridge = CvBridge()

    rospy.Subscriber("/zed/rgb/image_rect_color", Image, stereo_callback)

    rospy.loginfo("running")
    rospy.spin()
