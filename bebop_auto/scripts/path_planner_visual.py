#!/usr/bin/env python

#  --- Changelog ---
# Goal:     Input next gate position and obstacles and own position to plan a path through the target (eventually incorporating gate orientation)
# Status:   06/19: Creates a path with two waypoints: Own position and gate_position
#           07/06: Reads input from gate_detection and calculates global gate position. Publishes global gate position and orientation

import roslib
import sys
from geometry_msgs.msg import Pose
from bebop_auto.msg import Gate_Detection_Msg
from cv_bridge import CvBridge, CvBridgeError
import roslib
import math
#roslib.load_manifest('learning_tf')
import rospy
from tf import transformations as tfs
import common_resources as cr



#def eul2quat(z,y,x):
#    cz = math.cos(z * 0.5)
#    sz = math.sin(z * 0.5)
#    cx = math.cos(x * 0.5)
#    sx = math.sin(x * 0.5)
#    cy = math.cos(y * 0.5)
#    sy = math.sin(y * 0.5)
#    qw = cz * cx * cy + sz * sx * sy
#    qx = cz * sx * cy - sz * cx * sy
#    qy = cz * cx * sy + sz * sx * cy
#    qz = sz * cx * cy - cz * sx * sy
#    return [qx,qy,qz,qw]


def axang2quat(vector):
    length = math.sqrt(vector[0]*vector[0]+vector[1]*vector[1]+vector[2]*vector[2])
    s = math.sin(length / 2)
    x = vector[0]/length * s
    y = vector[1]/length * s
    z = vector[2]/length * s
    w = math.cos(length / 2)
    return[x, y, z, w]


def callback(data):
    msg = Pose()
    if data.rvec is not ():
        # bebop position and orientation
        bebop_pos = [data.bebop_pose.position.x, data.bebop_pose.position.y, data.bebop_pose.position.z]
        bebop_q = [data.bebop_pose.orientation.x, data.bebop_pose.orientation.y, data.bebop_pose.orientation.z, data.bebop_pose.orientation.w]
        # bebop_qi = [-bebop_q[0], -bebop_q[1], -bebop_q[2], bebop_q[3]]

        # gate position and orientation
        gate_pos = data.tvec
        gate_q = axang2quat(data.rvec)
        # gate_qi = [-gate_q[0], -gate_q[1], -gate_q[2], gate_q[3]]

        gate_global_p = cr.qv_mult(bebop_q, cr.qv_mult(cr.cam_q, gate_pos) + cr.BZ) + bebop_pos
        gate_global_q = tfs.quaternion_multiply(bebop_q, tfs.quaternion_multiply(cr.cam_q, gate_q))

        msg.position.x = gate_global_p[0]
        msg.position.y = gate_global_p[1]
        msg.position.z = gate_global_p[2]
        msg.orientation.w = gate_global_q[3]
        msg.orientation.x = gate_global_q[0]
        msg.orientation.y = gate_global_q[1]
        msg.orientation.z = gate_global_q[2]
    else:
        msg.position.x = None

    publisher.publish(msg)


def main():
    rospy.init_node('path_planner_visual', anonymous=True)

    global publisher
    rospy.Subscriber("/auto/gate_detection_result", Gate_Detection_Msg, callback)
    publisher = rospy.Publisher("/auto/path_visual", Pose, queue_size=2)

    rospy.spin()


if __name__ == '__main__':
    main()
