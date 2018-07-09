#!/usr/bin/env python

#  --- Changelog ---
# Goal:     Use bebop-position for granted every time it comes in and calculate a transformation from bebop to ZED odometry. In between, update global position based on differences the camera odometry provides
# Status:   06/19:  Simply passes through bebop odometry
#           07/03:  On a position update from bebop, it calculates the transformation from the bebop to the camera frame. On a camera update, it uses this transformation to calculate the current position of the drone


from tf import transformations as tfs
import rospy
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry
import math
import numpy as np
from scipy import linalg
import common_resources as cr


def bebop_update(data):
    global cRo, OC
    global zRc, CZ

    # calculate transformation from zed_origin to bebop_origin

    # position of bebop in bebop_origin coordinate system
    # bRo = transformations.euler_matrix(math.pi / 4, -math.pi / 3, math.pi / 5, 'rzyx')[:3, :3]
    # OB = np.array([[1.5], [13], [4]])
    bRo = tfs.quaternion_matrix(
        [data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z,
         data.pose.pose.orientation.w])[:3, :3]
    OB = np.array([[data.pose.pose.position.x], [data.pose.pose.position.y], [data.pose.pose.position.z]])

    if zRc is not None:
        print("new tf")
        # calculate position of camera_origin in bebop_origin coordinate system
        cRo = np.matmul(np.matmul(linalg.inv(zRc), cr.zRb), bRo)
        OC = OB + np.matmul(linalg.inv(bRo), cr.BZ) - np.matmul(linalg.inv(cRo), CZ)
        #cqo = tfs.quaternion_from_matrix(np.vstack((np.hstack((cRo, [[0], [0], [0]])), [0, 0, 0, 1])))

        #global odometry_tf_publisher
        #msg = Pose()
        #msg.position.x = OC[0]
        #msg.position.y = OC[1]
        #msg.position.z = OC[2]
        #msg.orientation.w = cqo[3]
        #msg.orientation.x = cqo[0]
        #msg.orientation.y = cqo[1]
        #msg.orientation.z = cqo[2]
        #odometry_tf_publisher.publish(msg)


def zed_update(data):
    global cRo, OC
    global zRc, CZ

    # calculate transformation from bebop_origin to bebop

    # position of zed camera in camera_origin coordinate system
    # zRc = transformations.euler_matrix(math.pi / 2, 0, 0, 'rzyx')[:3, :3]
    # CZ = np.array([[2.5], [-2], [0]])
    zRc = tfs.quaternion_matrix(
        [data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z,
         data.pose.pose.orientation.w])[:3, :3]
    CZ = np.array([[data.pose.pose.position.x], [data.pose.pose.position.y], [data.pose.pose.position.z]])

    if cRo is not None:
        print("update")
        # calculate position of bebop in bebop_origin coordinate system
        bRo = np.matmul(np.matmul(linalg.inv(cr.zRb), zRc), cRo)
        OB = OC + np.matmul(linalg.inv(cRo), CZ) - np.matmul(linalg.inv(bRo),  cr.BZ)
        bqo = tfs.quaternion_from_matrix(np.vstack((np.hstack((bRo, [[0], [0], [0]])), [0, 0, 0, 1])))

        global odometry_merged_publisher
        msg = Pose()
        msg.position.x = OB[0]
        msg.position.y = OB[1]
        msg.position.z = OB[2]
        msg.orientation.w = bqo[3]
        msg.orientation.x = bqo[0]
        msg.orientation.y = bqo[1]
        msg.orientation.z = bqo[2]
        odometry_merged_publisher.publish(msg)


def main():
    rospy.init_node('odometry_merger', anonymous=True)

    global cRo, OC
    global zRc, CZ
    cRo = None
    OC = None
    zRc = None
    CZ = None

    global odometry_merged_publisher
    #global odometry_tf_publisher
    odometry_merged_publisher = rospy.Publisher("/auto/odometry_merged", Pose, queue_size=2)
    #odometry_tf_publisher = rospy.Publisher("/auto/odometry_tf_o2c", Pose, queue_size=2)

    rospy.Subscriber("/bebop/odom", Odometry, bebop_update)
    rospy.Subscriber("/zed/odom", Odometry, zed_update)

    rospy.spin()


if __name__ == '__main__':
    main()
