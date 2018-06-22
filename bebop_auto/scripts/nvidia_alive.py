#!/usr/bin/env python
from __future__ import print_function

import rospy
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Bool

def talker():
    if not rospy.is_shutdown():
        rospy.init_node('nvidia_alive_node', anonymous=True)
        pub = rospy.Publisher('/auto/nvidia_alive', Bool, queue_size=1, latch=True)
        pub.publish(True)

if __name__ == '__main__':
    try:
        talker()
        while True:
            rospy.spin()
    except rospy.ROSInterruptException:
        pass
