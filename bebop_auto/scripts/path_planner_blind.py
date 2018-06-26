#!/usr/bin/env python

#  --- Changelog ---
# Goal:     Input current state and create a path using the information of the map and track layout. Used only if no gate was detected. Publishes path.
# Status:   06/19: Not existing
#           06/25: Takes in position and state
#           06/26: Creates a fake gate location at the very first time it is started and keeps publishing this location as blind path. Transform not working correctly


from geometry_msgs.msg import Pose, PoseArray, PoseStamped
from std_msgs.msg import Int32
# roslib.load_manifest('learning_tf')
import rospy
import tf
import time
import numpy as np


def position_updated(drone_pos):
    #print(drone_pos.position.x, drone_pos.position.y, drone_pos.position.z, drone_pos.orientation.x,
    #      drone_pos.orientation.y, drone_pos.orientation.z, drone_pos.orientation.w)

    global initialized
    if not initialized:
        print("initializing fake target")
        print("Drone position: ")
        print(drone_pos)
        q = [drone_pos.orientation.x, drone_pos.orientation.y, drone_pos.orientation.z, drone_pos.orientation.w]
        qi = q
        qi[3] = -qi[3]

        gate = [1, 0, 1, 0]
        print("Virtual gate (local):")
        print(gate[0:3])
        gate_tf = tf.transformations.quaternion_multiply(qi, gate)
        gate_tf = gate_tf.tolist()[0:3]
        print("Virtual gate (global):")
        print(gate_tf)

        # initialize with this position
        global target
        target = gate_tf
        initialized = True

    if current_state == 4:
        WP = Pose()
        WP.x = target[0]
        WP.y = target[1]
        WP.z = target[2]

        global publisher
        publisher.publish(Pose)


def state_updated(data):
    global current_state
    global target
    current_state = data.data


def main():
    rospy.init_node('path_planner_visual', anonymous=True)

    global current_state
    current_state = -1
    rospy.Subscriber("/auto/state_machine", Int32, state_updated)
    rospy.Subscriber("/auto/odometry_merged", Pose, position_updated)
    global publisher
    publisher = rospy.Publisher("/auto/path_blind", Pose, queue_size=2)

    global initialized
    initialized = False

    rospy.spin()


if __name__ == '__main__':
    main()
