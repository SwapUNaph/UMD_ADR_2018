#!/usr/bin/env python

#  --- Changelog ---
# Goal:     Use bebop-position for granted every time it comes in and calculate a transformation from bebop to ZED odometry. In between, update global position based on differences the camera odometry provides
# Status:   06/19:  Simply passes through bebop odometry
#           06/25:

from __future__ import print_function
import tf
import sys
import rospy
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry




def bebop_update(data):
    global latest_zed_odo
    global latest_tf
    latest_bebop_odo = data.pose.pose

    # calculate transformation from zed to bebop


    tl = tf.TransformListener()
    tf_bebop = tf.TransformerROS.fromTranslationRotation(tl, latest_bebop_odo.position, latest_bebop_odo.orientation)
    tf_zed = tf.TransformerROS.fromTranslationRotation(tl, latest_zed_odo.position, latest_zed_odo.orientation)





def zed_update(data):
    global odometry_merged
    global latest_tf
    global latest_zed_odo
    latest_zed_odo = data.pose.pose

    # transform current zed to bebop
    # update tf and odo_merged






    global odometry_merged_publisher
    # rospy.loginfo(rospy.get_caller_id() + "\nI heard %s", data)

    # print("odometry: ...")
    # print[data.pose.pose.position.x, data.pose.pose.position.y, data.pose.pose.position.z]
    # print[data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w]
    # print[data.twist.twist.linear.x, data.twist.twist.linear.y, data.twist.twist.linear.z]
    # print[data.twist.twist.angular.x, data.twist.twist.angular.y, data.twist.twist.angular.z]
    # print("---")

    msg = data.pose.pose
    odometry_merged_publisher.publish(msg)


def main():
    rospy.init_node('odometry_merger', anonymous=True)

    global latest_tf
    global odometry_merged
    global latest_zed_odo
    latest_tf = None
    odometry_merged = None
    latest_zed_odo = None

    global odometry_merged_publisher
    odometry_merged_publisher = rospy.Publisher("/auto/odometry_merged", Pose, queue_size=2)

    rospy.Subscriber("/bebop/odom", Odometry, bebop_update)
    rospy.Subscriber("/zed/odom", Odometry, zed_update)

    rospy.spin()


if __name__ == '__main__':
    main()
