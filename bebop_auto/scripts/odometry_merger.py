#!/usr/bin/env python

#  --- Changelog ---
# Goal:     Use Kalman filter to estimate current position based on bebop odometry (high accuracy, low frequency) and stereo odometry (low accuracy, high frequency)
# Status:   06/19:  Simply passes through bebop odometry
#           06/25:

from __future__ import print_function

import sys
import rospy
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge, CvBridgeError
#

def callback(data):
    # rospy.loginfo(rospy.get_caller_id() + "\nI heard %s", data)

    # print("odometry: ...")
    # print[data.pose.pose.position.x, data.pose.pose.position.y, data.pose.pose.position.z]
    # print[data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w]
    # print[data.twist.twist.linear.x, data.twist.twist.linear.y, data.twist.twist.linear.z]
    # print[data.twist.twist.angular.x, data.twist.twist.angular.y, data.twist.twist.angular.z]
    # print("---")

    msg = data.pose.pose
    publisher.publish(msg)


def main():
    rospy.init_node('odometry_merger', anonymous=True)
    publisher = rospy.Publisher("/auto/odometry_merged", Pose, queue_size=2)
    rospy.Subscriber("/bebop/odom", Odometry, callback)

    rospy.spin()

    print("Shutting down")
    # cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
