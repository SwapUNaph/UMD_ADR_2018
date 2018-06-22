#!/usr/bin/env python
from __future__ import print_function

import rospy
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import String

def talker():
    pub = rospy.Publisher('/nvidia_alive', String, queue_size=10)
    rospy.init_node('nvidia_alive_node', anonymous=True)
    rate = rospy.Rate(1) # 1 Hz
    while not rospy.is_shutdown():
        hello_str = "Nvidia is alive: %s" % rospy.get_time()
        pub.publish(hello_str)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
