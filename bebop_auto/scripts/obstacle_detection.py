#!/usr/bin/env python
from __future__ import print_function

import sys
import rospy
from geometry_msgs.msg import PoseArray, Pose
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2 as cv



def callbackPose(data):
    global vehicle_pose
    vehicle_pose = data


def callback(data):
    global vehicle_pose
    try:
        depth = self.bridge.imgmsg_to_cv2(data, "CV_16UC1")
    except CvBridgeError as e:
        print(e)

    cv.imshow('Raw Depth', depth)
    th = cv.threshold(depth,127,255,cv.THRESH_BINARY)
    cv.imshow('Threshold', th)
    

    # keypoints = detector.detect(im)





    poses = PoseArray()
    poses.poses = []

    publisher.publish(msg)




if __name__ == '__main__':
    # main(sys.argv)

    rospy.init_node('odometry_merger', anonymous=True)
    publisher = rospy.Publisher("/obstacles", PoseArray, queue_size=2)

    vehicle_pose = None
    # detector = cv2.SimpleBlobDetector()

    rospy.Subscriber("/odometry_merged", Pose, callbackPose)
    rospy.Subscriber("/camera/pub_depth", Image, callbackDepth)


    rospy.spin()

    print("Shutting down")
    # cv2.destroyAllWindows()
