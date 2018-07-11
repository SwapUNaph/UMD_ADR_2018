#!/usr/bin/env python

#  --- Changelog ---
# Goal:     Input next gate position and obstacles and own position to plan a path through the target (eventually incorporating gate orientation)
# Status:   06/19: Creates a path with two waypoints: Own position and gate_position
#           07/06: Reads input from gate_detection and calculates global gate position. Publishes global gate position and orientation

import roslib
import sys
from geometry_msgs.msg import Pose
from bebop_auto.msg import Gate_Detection_Msg
from bebop_auto.msg import Waypoint_Msg
from std_msgs.msg import Float32MultiArray, Int32
from cv_bridge import CvBridge, CvBridgeError
import roslib
import math
#roslib.load_manifest('learning_tf')
import rospy
from tf import transformations as tfs
import common_resources as cr
import time
import signal


def signal_handler(signal, frame):
    sys.exit(0)


def axang2quat(vector):
    length = math.sqrt(vector[0]*vector[0]+vector[1]*vector[1]+vector[2]*vector[2])
    s = math.sin(length / 2)
    x = vector[0]/length * s
    y = vector[1]/length * s
    z = vector[2]/length * s
    w = math.cos(length / 2)
    return[x, y, z, w]


def find_average(latest_gates):
    # transpose latest_gates
    count = len(latest_gates)
    [x, y, z, angles] = map(list, zip(*latest_gates))

    sin_sum = 0
    cos_sum = 0
    for angle in angles:
        sin_sum = sin_sum + math.sin(angle)
        cos_sum = cos_sum + math.cos(angle)
    average_angle = math.atan2(sin_sum, cos_sum)

    return [sum(x)/count, sum(y)/count, sum(z)/count, average_angle]


def callback(data):
    global latest_gates

    if False:#data.rvec is not ():
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
        gate_normal_vec = cr.qv_mult(gate_global_q, [0, 0, 1])
        heading_to_gate = math.atan2(gate_pos[1]-bebop_pos[1],gate_pos[0]-bebop_pos[0])
        heading_of_gate = math.atan2(gate_normal_vec[1],gate_normal_vec[0])
        heading_difference = math.fabs(heading_to_gate - heading_of_gate)*180/math.pi
        if 90 < heading_difference < 270:
            if heading_of_gate < 0:
                heading_of_gate = heading_of_gate + math.pi
            else:
                heading_of_gate = heading_of_gate - math.pi

        distance = math.sqrt(
              (gate_global_p[0] - latest_gates[-1][0]) * (gate_global_p[0] - latest_gates[-1][0])
            + (gate_global_p[1] - latest_gates[-1][1]) * (gate_global_p[1] - latest_gates[-1][1])
            + (gate_global_p[2] - latest_gates[-1][2]) * (gate_global_p[2] - latest_gates[-1][2]))
        if distance > 0.5:
            latest_gates = [gate_global_p + [heading_of_gate]]
        else:
            latest_gates.append(gate_global_p + [heading_of_gate])
            if len(latest_gates) > 5:
                del latest_gates[0]

        average_gate = find_average(latest_gates)
    else:
        average_gate = [0, 0, 0, None]
    # publisher.publish(average_gate)


    msg = Waypoint_Msg()

    global current_state
    global fake_visual_target
    global start_time
    if current_state == 4:
        if start_time is None:
            start_time = time.time()

        if fake_visual_target is None:
            global current_position
            bebop_pos = [current_position.position.x, current_position.position.y, current_position.position.z]
            bebop_q = [current_position.orientation.x, current_position.orientation.y, current_position.orientation.z,
                       current_position.orientation.w]
            virtual_gate = [1,1,0]
            fake_visual_target = cr.qv_mult(bebop_q,virtual_gate) + bebop_pos
            fake_visual_target = fake_visual_target.tolist()

        if start_time + 15 < time.time():
            msg.pos = fake_visual_target
            msg.hdg = 0

    publisher.publish(msg)


def update(data, args):
    if args == "state":
        global current_state
        current_state = data.data
    elif args == "position":
        global current_position
        current_position = data


def main():
    signal.signal(signal.SIGINT, signal_handler)
    rospy.init_node('waypoint_creation_visual', anonymous=True)

    global start_time
    global fake_visual_target
    start_time = None
    fake_visual_target = None

    global current_state
    global current_position
    current_state = -1
    rospy.Subscriber("/auto/state_machine", Int32, update, "state")
    rospy.Subscriber("/auto/odometry_merged", Pose, update, "position")

    global latest_gates
    latest_gates = [[0,0,0,0]]
    global publisher
    rospy.Subscriber("/auto/gate_detection_result", Gate_Detection_Msg, callback)
    publisher = rospy.Publisher("/auto/wp_visual", Waypoint_Msg, queue_size=1)

    rate = rospy.Rate(20)
    while True:
        callback(None)
        rate.sleep()

    # rospy.spin()


if __name__ == '__main__':
    main()
