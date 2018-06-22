#!/usr/bin/env python
from __future__ import print_function

import roslib
#roslib.load_manifest('my_package')
import sys
from geometry_msgs.msg import Pose
from cv_bridge import CvBridge, CvBridgeError
import roslib
#roslib.load_manifest('learning_tf')
import rospy
import tf




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
#
# def main(args):
#     ic = image_converter()
#     rospy.init_node('image_converter', anonymous=True)
#     try:
#         rospy.spin()
#     except KeyboardInterrupt:
#         print("Shutting down")
#     cv2.destroyAllWindows()


def gate_updated(data):
    global gate_data
    gate_data = data


def position_updated(pos_data):
    global gate_data
    if gate_data is not None:
        #pos_tf = tf.transformations.poseMsgToTF(pos_data)
        #gate_tf = tf.transformations.poseMsgToTF(gate_data)
        print(gate_tf.inverseTimes(pos_tf))

        WP = [[pos_data.position.x, pos_data.position.y, pos_data.position.z],
              [gate_data.position.x, gate_data.position.y, gate_data.position.z]]

        publisher.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))


        gate_data = None


if __name__ == '__main__':
    #main(sys.argv)

    rospy.init_node('path_planner', anonymous=True)

    gate_data = None
    gate_position_sub = rospy.Subscriber("/auto/global_gate_position", Pose, gate_updated)
    own_position_sub = rospy.Subscriber("/auto/odometry_merged", Pose, position_updated)
    publisher = rospy.Publisher("/auto/path_planned", Pose, queue_size=2)

    rospy.spin()

    print("Shutting down")
    #cv2.destroyAllWindows()
