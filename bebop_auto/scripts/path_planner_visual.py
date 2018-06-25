#!/usr/bin/env python

#  --- Changelog ---
# Goal:     Input next gate position and obstacles and own position to plan a path through the target (eventually incorporating gate orientation)
# Status:   06/19: Creates a path with two waypoints: Own position and gate_position
#           06/25:

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


def gate_updated(gate_pos):
    global drone_pos

    if drone_pos is not None:
        #drone_tf = tf.transformations.poseMsgToTF(drone_pos)
        #gate_tf = tf.transformations.poseMsgToTF(gate_pos)
        #print(gate_tf.inverseTimes(pos_tf))

        WP = [[drone_pos.position.x, drone_pos.position.y, drone_pos.position.z],
              [gate_pos.position.x, gate_pos.position.y, gate_pos.position.z]]

        #publisher.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))

        drone_pos = None

def position_updated(data):
    global drone_pos
    drone_pos = data


def main():
    rospy.init_node('path_planner_visual', anonymous=True)

    drone_pos = None
    rospy.Subscriber("/auto/global_gate_position", Pose, gate_updated)
    rospy.Subscriber("/auto/odometry_merged", Pose, position_updated)
    publisher = rospy.Publisher("/auto/path_planned", Pose, queue_size=2)

    rospy.spin()

    print("Shutting down")
    #cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
