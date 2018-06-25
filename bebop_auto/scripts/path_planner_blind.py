#!/usr/bin/env python

#  --- Changelog ---
# Goal:     Input current state and create a path using the information of the map and track layout. Used only if no gate was detected. Publishes path.
# Status:   06/19: Not existing
#           06/25: Takes in position and state

from __future__ import print_function
from geometry_msgs.msg import Pose, PoseArray, PoseStamped
from std_msgs.msg import Int32
# roslib.load_manifest('learning_tf')
import rospy
import tf


def position_updated(drone_pos):
    print(drone_pos)

    global initialized
    if not initialized:
        #initialize with this position
        initialized = True

    if current_state == 4:
        WP = [drone_pos.position.x, drone_pos.position.y, drone_pos.position.z]
        # publisher.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))


def state_updated(data):
    global current_state
    current_state = data.data


def main():
    rospy.init_node('path_planner_visual', anonymous=True)

    current_state = -1
    rospy.Subscriber("/auto/state_machine", Int32, state_updated)
    rospy.Subscriber("/auto/odometry_merged", Pose, position_updated)
    publisher = rospy.Publisher("/auto/path_blind", Pose, queue_size=2)

    global initialized
    initialized = False

    rospy.spin()

    print("Shutting down")
    # cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
