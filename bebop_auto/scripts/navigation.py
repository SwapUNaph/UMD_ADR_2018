#!/usr/bin/env python

#  --- Changelog ---
# Goal:     Input of both path planners. Find relevant waypoint depending on current position. Use visual path if possible, otherwise use blind backup. Calculate driving command based on waypoint position and orientation. Create driving commands
# Status:   06/19: Not existing
#           06/26: Reads updates from blind path, visual path and position. Publishes empty command

from geometry_msgs.msg import Pose, PoseArray, PoseStamped
from std_msgs.msg import Int32
# roslib.load_manifest('learning_tf')
import rospy
import tf
import math
import time
import signal
import sys
import numpy as np
from bebop_auto.msg import Auto_Driving_Msg
from pid_Function import PID 


def signal_handler(signal, frame):
    sys.exit(0)


def qv_mult(q1, v1):
    length = math.sqrt(v1[0] * v1[0] + v1[1] * v1[1] + v1[2] * v1[2])
    if length != 0:
        v1 = tf.transformations.unit_vector(v1)
    q2 = list(v1)
    q2.append(0.0)
    return tf.transformations.quaternion_multiply(tf.transformations.quaternion_multiply(q1, q2),
                                                  tf.transformations.quaternion_conjugate(q1))[:3] * length


def received_update(data, args):
    global drone
    global path_visual
    global path_blind
    global state_machine

    if args == "position":
        drone = data
    elif args == "path_visual":
        path_visual = data
    elif args == "path_blind":
        path_blind = data
    elif args == "state_machine":
        state_machine = data.data


def main():
    # Enable killing the script with Ctrl+C.
    signal.signal(signal.SIGINT, signal_handler)

    rospy.init_node('navigation', anonymous=True)

    global drone
    global path_visual
    global path_blind
    global state_machine
    drone = None
    path_visual = None
    path_blind = None
    state_machine = -1
    rospy.Subscriber("/auto/odometry_merged",   Pose, received_update, "position")
    rospy.Subscriber("/auto/path_blind",        Pose, received_update, "path_blind")
    rospy.Subscriber("/auto/path_visual",       Pose, received_update, "path_visual")
    rospy.Subscriber("/auto/state_machine",    Int32, received_update, "state_machine")

    driver_publisher = rospy.Publisher("/auto/auto_drive", Auto_Driving_Msg, queue_size=1)

    Z_PID = PID()
    R_PID = PID()
    F_PID = PID()
    T_PID = PID()

    # Wait until connecction between ground and air is established. Script can get stuck here
    while state_machine <= 1:
        rospy.loginfo("waiting")
        time.sleep(2)

    rate = rospy.Rate(20)

    while True:
        rate.sleep()

        # set applicable path
        if path_visual is None:
            if path_blind is not None:
                path = path_blind
            else:
                # rospy.loginfo("no path")
                continue
        else:
            path = path_visual

        if drone is None:
            # rospy.loginfo("no position")
            continue



        dy = path.position.y - drone.position.y
        dx = path.position.x - drone.position.x
        dist = (dy**2+dx**2)**.5
        
        euler = tf.transformations.euler_from_quaternion(drone.orientation)

        gate_theta = tf.transformations.euler_from_quaternion(path.orientation)

        vehicle_theta = math.atan2(dy,dx)
        d_theta = gate_theta-vehicle_theta

        t_error = -dist*math.sin(d_theta)
        f_error = -dist*math.cos(d_theta)
        z_error = path.position.z - drone.position.z
        r_error = vehicle_theta - euler[3]



        # calculate path to WP
        diff_global = [path.position.x - drone.position.x,
                       path.position.y - drone.position.y,
                       path.position.z - drone.position.z]

        msg = Auto_Driving_Msg()

        # implementation exactly reverse as in matlab. Invert necessary when not required in matlab vice versa
        q = [drone.orientation.x, drone.orientation.y, drone.orientation.z, drone.orientation.w]
        qi = [-q[0], -q[1], -q[2], q[3]]

        diff_bebop = qv_mult(qi, diff_global)
        # print("heading to goal " + str(math.atan2(-diff_bebop[1], diff_bebop[0]) * 180 / math.pi))

        # change driving message
        limit = 0.2
        gain = 0.2/0.5

        # msg.x = max(min(F_PID.update(f_error), 1),-1)
        msg.x = 0

        msg.y = max(min(T_PID.update(t_error), 1),-1)
        # msg.x = 0

        # msg.z = max(min(Z_PID.update(z_error), 1),-1)
        msg.x = 0

        # msg.r = max(min(R_PID.update(z_error), 1),-1)
        msg.x = 0

        #rospy.loginfo("fwd: " + "{:.9f}".format(msg.x) + " | left: " + "{.9f}".format(msg.y) + " | up: " + "{.9f}".format(
        #    msg.z) + " | ccw: " + "{.9f}".format(msg.r))
        driver_publisher.publish(msg)


if __name__ == '__main__':
    main()
