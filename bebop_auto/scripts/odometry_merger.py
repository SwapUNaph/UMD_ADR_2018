#!/usr/bin/env python
from __future__ import print_function

import sys
import rospy
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge, CvBridgeError
#
# class image_converter:
#
#     def __init__(self):
#         #self.image_pub = rospy.Publisher("image_topic_2",Image)
#
#         self.bridge = CvBridge()
#         self.image_sub = rospy.Subscriber("/bebop/image_raw",Image,self.callback)
#
#     def callback(self,data):
#         try:
#             cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
#         except CvBridgeError as e:
#             print(e)
#
#         (rows,cols,channels) = cv_image.shape
#         if cols > 60 and rows > 60 :
#             cv2.circle(cv_image, (50,50), 10, 255)
#
#         cv2.imshow("Image window", cv_image)
#         cv2.waitKey(3)
#
#         #try:
#         #    self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
#         #except CvBridgeError as e:
#         #    print(e)
#
# def main(args):
#     ic = image_converter()
#     rospy.init_node('image_converter', anonymous=True)
#     try:
#         rospy.spin()
#     except KeyboardInterrupt:
#         print("Shutting down")
#     cv2.destroyAllWindows()


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


if __name__ == '__main__':
    # main(sys.argv)

    rospy.init_node('odometry_merger', anonymous=True)
    publisher = rospy.Publisher("odometry_merged", Pose, queue_size=2)
    rospy.Subscriber("/bebop/odom", Odometry, callback)

    rospy.spin()

    print("Shutting down")
    # cv2.destroyAllWindows()
