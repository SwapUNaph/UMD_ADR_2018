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
import math
import numpy as np


def qv_mult(q1, v1):
    length = math.sqrt(v1[0]*v1[0]+v1[1]*v1[1]+v1[2]*v1[2])
    v1 = tf.transformations.unit_vector(v1)
    q2 = list(v1)
    q2.append(0.0)
    return tf.transformations.quaternion_multiply(tf.transformations.quaternion_multiply(q1, q2),
                                                tf.transformations.quaternion_conjugate(q1))[:3]*length


def position_updated(drone_pos):
    global fake_target_set
    global publisher

    if not fake_target_set:
        rospy.loginfo("set fake target")

        # implementation exactly reverse as in matlab. Invert necessary when not required in matlab vice versa
        q = [drone_pos.orientation.x, drone_pos.orientation.y, drone_pos.orientation.z, drone_pos.orientation.w]
        qi = [-q[0], -q[1], -q[2], q[3]]

        # own heading:
        # dx0 = [1, 0, 0]
        # dy0 = [0, 1, 0]
        # dz0 = [0, 0, 1]
        # dx = qv_mult(q, dx0)
        # dy = qv_mult(q, dy0)
        # dz = qv_mult(q, dz0)

        gate = [-0.5, 0, 1.5]
        gate_tf = qv_mult(q, gate)
        gate_tf = gate_tf.tolist()
        rospy.loginfo("global position: " + str(gate_tf))

        # initialize with this position
        global target
        target = gate_tf
        fake_target_set = True

    if current_state <= 3:
        # publish zeros before mission start
        publisher.publish(Pose())

    if current_state == 4:
        wp = Pose()
        wp.position.x = target[0]
        wp.position.y = target[1]
        wp.position.z = target[2]

        publisher.publish(wp)


def state_updated(data):
    global current_state
    global target
    current_state = data.data


def main():
    rospy.init_node('path_planner_blind', anonymous=True)

    global current_state
    current_state = -1
    rospy.Subscriber("/auto/state_machine", Int32, state_updated)
    rospy.Subscriber("/auto/odometry_merged", Pose, position_updated)
    global publisher
    publisher = rospy.Publisher("/auto/path_blind", Pose, queue_size=2)

    global fake_target_set
    fake_target_set = False

    rospy.spin()


if __name__ == '__main__':
    main()
