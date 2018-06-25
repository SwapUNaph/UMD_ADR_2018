#!/usr/bin/env python

#  --- Changelog ---
# Goal:     Input current state and create a path using the information of the map and track layout. Used only if no gate was detected. Publishes path.
# Status:   06/19: Not existing
#           06/25: Takes in position and state

from __future__ import print_function

import roslib
# roslib.load_manifest('my_package')
import sys
from geometry_msgs.msg import Pose
from cv_bridge import CvBridge, CvBridgeError
import roslib
# roslib.load_manifest('learning_tf')
import rospy
import tf


def position_updated(drone_pos):
    if current_state == 4:
        WP = [drone_pos.position.x, drone_pos.position.y, drone_pos.position.z]
        # publisher.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))


def state_updated(data):
    global current_state
    current_state = data.data


def main():
    rospy.init_node('path_planner_visual', anonymous=True)

    current_state = -1
    rospy.Subscriber("/auto/state_machine", Pose, state_updated)
    rospy.Subscriber("/auto/odometry_merged", Pose, position_updated)
    publisher = rospy.Publisher("/auto/path_blind", Pose, queue_size=2)

    rospy.spin()

    print("Shutting down")
    # cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
