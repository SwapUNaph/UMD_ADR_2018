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
import time
import numpy as np
from bebop_auto.msg import Auto_Driving_Msg


def received_update(data, args):
    #print(drone_pos.position.x, drone_pos.position.y, drone_pos.position.z, drone_pos.orientation.x,
    #      drone_pos.orientation.y, drone_pos.orientation.z, drone_pos.orientation.w)

    global position
    global path_visual
    global path_blind

    if args == "position":
        position = data
    elif args == "path_visual":
        path_visual = data
    elif args == "path_blind":
        path_blind = data


    print("send driving message")
    msg = Auto_Driving_Msg()
    msg.x = 0
    msg.y = 0
    msg.z = 0
    msg.r = 0
    publisher.publish(msg)


def main():
    rospy.init_node('navigation', anonymous=True)

    global position
    global path_visual
    global path_blind
    rospy.Subscriber("/auto/odometry_merged", Pose, received_update, "position")
    rospy.Subscriber("/auto/path_blind", Pose, received_update, "path_blind")
    rospy.Subscriber("/auto/path_visual", Pose, received_update, "path_visual")

    global publisher
    publisher = rospy.Publisher("/auto/auto_drive", Auto_Driving_Msg, queue_size=1)

    rospy.spin()


# read in position and path
# if we are close enough to the last waypoint, advance status



if __name__ == '__main__':
    main()

