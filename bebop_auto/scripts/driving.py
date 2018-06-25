# Input from ground or from navigation. Ground overrides navigation. Publish to drone.



#!/usr/bin/env python
from __future__ import print_function

import sys
import rospy
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge, CvBridgeError




def callbackNavigation(data):
	if not Override:
    	publisher.publish(data)


def callbackGround(data):
	if not Override:
        Overide = True
        print('Command Overriden from Ground Station')
    cmd_vel_pub.publish(data)


if __name__ == '__main__':
    # main(sys.argv)

    rospy.init_node('driving', anonymous=True)
    cmd_vel_pub = rospy.Publisher("/bebop/cmd_vel", Twist, queue_size=2)
    
   	Override = False

    rospy.Subscriber("/navigation", Twist, callbackNavigation)
    rospy.Subscriber("/ground_output", Twist, callbackGround)


    rospy.spin()

    print("Shutting down")
    # cv2.destroyAllWindows()
