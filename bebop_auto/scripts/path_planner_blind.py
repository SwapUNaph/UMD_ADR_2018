#!/usr/bin/env python

#  --- Changelog ---
# Goal:     Input current state and create a path using the information of the map and track layout. Used only if no gate was detected. Publishes path.
# Status:   06/19: Not existing
#           06/25: Copy of path_planner_visual

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

    rospy.init_node('path_planner_blind', anonymous=True)

    gate_data = None
    gate_position_sub = rospy.Subscriber("/auto/global_gate_position", Pose, gate_updated)
    own_position_sub = rospy.Subscriber("/auto/odometry_merged", Pose, position_updated)
    publisher = rospy.Publisher("/auto/path_planned", Pose, queue_size=2)

    rospy.spin()

    print("Shutting down")
    #cv2.destroyAllWindows()
