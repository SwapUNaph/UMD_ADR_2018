#!/usr/bin/env python

#  --- Changelog ---
# Goal:     Display all relevant flight data like position, waypoint, state, video, battery level, wifi status,
# Status:   06/19: There is a gui that displays some information.
#           06/25:

import rospy
from geometry_msgs.msg import Twist, Pose, PoseStamped, PointStamped, Point
from std_msgs.msg import Empty, Int32, Float32
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from bebop_auto.msg import Gate_Detection_Msg, WP_Msg, Auto_Driving_Msg
from bebop_msgs.msg import Ardrone3CameraStateOrientation
from bebop_msgs.msg import CommonCommonStateBatteryStateChanged
from bebop_msgs.msg import CommonCommonStateWifiSignalChanged
from bebop_msgs.msg import Ardrone3PilotingStateAltitudeChanged
from bebop_msgs.msg import Ardrone3PilotingStateAttitudeChanged
from bebop_msgs.msg import Ardrone3PilotingStateSpeedChanged
#from bebop_msgs.msg import OverHeatStateOverHeatChanged

from visualization_msgs.msg import Marker, MarkerArray

import cv2
import numpy as np
import time
import math
import signal
import sys
import tf

def signal_handler(signal, frame):
    sys.exit(0)

class bebop_data:
    def __init__(self):        

        self.vichile_pub = rospy.Publisher("/auto/rviz/vehicle", MarkerArray, queue_size=1)

        self.gate_pub = rospy.Publisher("/auto/rviz/gate", MarkerArray, queue_size=1)

        self.gate_number = 0
        self.state_level = 9
        self.gate_arm_theta = 0.0

        self.battery_level = 0
        self.gate_size = 1.4
        
        # self.gate_visual_pub = rospy.Publisher("/auto/rviz/gate_visual", MarkerArray, queue_size=1)
        # self.gate_blind_pub = rospy.Publisher("/auto/rviz/gate_blind", MarkerArray, queue_size=1)
        # self.gate_current = rospy.Publisher("/auto/rviz/gate_current", MarkerArray, queue_size=1)
        # self.gate_look_pub = rospy.Publisher("/auto/rviz/gate_look", MarkerArray, queue_size=1)
        # self.command_viz_pub = rospy.Publisher("/auto/rviz/commands", MarkerArray, queue_size=1)


        # static_transform_publisher x y z qx qy qz qw frame_id child_frame_id  period_in_ms
        self.tbr = tf.TransformBroadcaster()



        rospy.Subscriber("/bebop/camera_control", Twist, self.callback, "cam_ctrl")
        rospy.Subscriber("/bebop/reset", Empty, self.callback, "reset")
        rospy.Subscriber("/zed/odom", Odometry, self.callback, "zed_odom")
        rospy.Subscriber("/bebop/odom", Odometry, self.callback, "odom")
        rospy.Subscriber("/auto/auto_drive", Auto_Driving_Msg, self.callback, "cmds")
        rospy.Subscriber("/bebop/states/ardrone3/CameraState/Orientation", Ardrone3CameraStateOrientation, self.callback,"cam_orient")
        rospy.Subscriber("/bebop/states/common/CommonState/BatteryStateChanged", CommonCommonStateBatteryStateChanged,self.callback, "battery")
        rospy.Subscriber("/bebop/states/common/CommonState/WifiSignalChanged", CommonCommonStateWifiSignalChanged,self.callback, "wifi")
        rospy.Subscriber("/bebop/states/ardrone3/PilotingState/AltitudeChanged", Ardrone3PilotingStateAltitudeChanged,self.callback, "altitude")
        rospy.Subscriber("/bebop/states/ardrone3/PilotingState/AttitudeChanged", Ardrone3PilotingStateAttitudeChanged,self.callback, "attitude")
        # rospy.Subscriber("/bebop/states/ardrone3/PilotingState/PositionChanged", Ardrone3PilotingStatePositionChanged,  callback, "position") no updates
        rospy.Subscriber("/bebop/states/ardrone3/PilotingState/SpeedChanged", Ardrone3PilotingStateSpeedChanged,self.callback, "speed")

        # rospy.Subscriber("/bebop/states/common/OverHeatState/OverHeatChanged",   OverHeatStateOverHeatChanged,          callback, "overheat")
        # ~states/enable_pilotingstate_flyingstatechanged parameter to true
        # enable the publication of flying state changes to topic states/ARDrone3/PilotingState/FlyingStateChanged

        # rospy.Subscriber("/auto/gate_detection_gate", Image,self.callback, 'Image')
        rospy.Subscriber("/bebop/image_raw", Image,self.callback, 'Image')
        rospy.Subscriber("/auto/wp_visual", WP_Msg, self.callback,'wp_visual')
        rospy.Subscriber("/auto/wp_current", WP_Msg, self.callback,'wp_current')
        rospy.Subscriber("/auto/wp_blind", WP_Msg, self.callback,'wp_blind')
        rospy.Subscriber("/auto/wp_look", WP_Msg, self.callback,'wp_look')
        rospy.Subscriber("/auto/state_auto", Int32, self.callback,'state')
        rospy.Subscriber("/auto/state_auto", Float32, self.callback,'state')

        rospy.Subscriber("/auto/gate_size", Float32, self.callback,'gate_size')


        # rospy.Subscriber("/auto/state", Int32,self.callback, "state")        




    def callback(self,data,args):
        # rospy.loginfo(rospy.get_caller_id() + "\nI heard %s", data)
        if args == "gate_size":
            self.gate_size = data.data
        if args == "cam_ctrl":
            # canvas.itemconfig(cam_text)
            # canvas.coords(cam_ptr, 25 * (data.angular.z / 60 + 1), 25 * (-data.angular.y / 90 + 1))
            pass
        elif args == "reset":
            # print("reset pressed")
            pass

        elif args == "arm_theta":
            self.gate_arm_theta = data.data

        elif args == "state":
            if data.data != self.state_level:
                self.gate_number = self.gate_number+1
                # self.state_level = self.state_level+10

        elif args == "odom":
            quat = data.pose.pose.orientation
            # print quat
            pos = data.pose.pose.position
            # quat = tf.transformations.quaternion_from_euler(quat.x,quat.y,quat.z,quat.w)

            
            marker_array = MarkerArray()

            point_marker = Marker()
            point_marker.header.frame_id = "vehicle_frame"
            point_marker.header.stamp    = rospy.get_rostime()
            point_marker.ns = "vehicle"
            point_marker.id = 0
            point_marker.type = 1 # prism
            point_marker.action = 0
            point_marker.scale.x = .2
            point_marker.scale.y = .05
            point_marker.scale.z = .05
            point_marker.pose.orientation.w = 1
            # point_marker.color.r = 0
            # point_marker.color.g = 0
            point_marker.color.b = 1
            point_marker.color.a = 1.0
            point_marker.lifetime = rospy.Duration(0)
            marker_array.markers.append(point_marker)

            point_marker.color.r = .5
            point_marker.color.g = .5
            point_marker.color.b = .5
            point_marker.type = 3
            point_marker.pose.position.z = .02
            point_marker.pose.position.x = .1
            point_marker.pose.position.y = .1
            point_marker.scale.x = .02
            point_marker.scale.y = .12
            point_marker.scale.z = .12
            marker_array.markers.append(point_marker)

            point_marker.pose.position.y = -.1
            marker_array.markers.append(point_marker)
            
            point_marker.pose.position.x = -.1
            marker_array.markers.append(point_marker)
            
            point_marker.pose.position.y = .1
            marker_array.markers.append(point_marker)
            
            self.vichile_pub.publish(marker_array)
            self.tbr.sendTransform((pos.x,pos.y,pos.z),(quat.x,quat.y,quat.z,quat.w),rospy.get_rostime(),'vehicle_frame', "odom")
            
            
        elif args == "zed_odom":
            quat = data.pose.pose.orientation
            # print quat
            pos = data.pose.pose.position
            # quat = tf.transformations.quaternion_from_euler(quat.x,quat.y,quat.z,quat.w)

            
            marker_array = MarkerArray()

            point_marker = Marker()
            point_marker.header.frame_id = "zed_frame"
            point_marker.header.stamp    = rospy.get_rostime()
            point_marker.ns = "camera"
            point_marker.id = 0
            point_marker.type = 2 # sphere
            point_marker.action = 0
            point_marker.scale.x = .4
            point_marker.scale.y = .2
            point_marker.scale.z = .1
            point_marker.pose.orientation.w = 1
            point_marker.color.r = .5
            point_marker.color.g = .5
            point_marker.color.b = 1
            point_marker.color.a = 1.0
            point_marker.lifetime = rospy.Duration(0)

            marker_array.markers.append(point_marker)

            self.vichile_pub.publish(marker_array)
            self.tbr.sendTransform((pos.x,pos.y,pos.z),(quat.x,quat.y,quat.z,quat.w),rospy.get_rostime(),'zed_frame', "odom")
            
            
        elif args == "cam_orient":
            pass
        elif args == "battery":

            # print ' '
            # print 'Battery level: ',data.percent
            # print ' '
            self.battery_level = data.percent
        elif args == "wifi":
            if data.rssi * 2 + 160 < 100.0:
                print 'R: ',(data.rssi * 2 + 160),'  B: ',self.battery_level
            else:
                print 'R: ',(data.rssi * 2 + 160),' B: ',self.battery_level
        elif args == "altitude":
            # print "altitude ",data.altitude
            pass
        elif args == "attitude":
            # print "attitude", print [data.roll, data.pitch, data.yaw]
            pass
        elif args == "speed":
            # print "speed", [data.speedX, data.speedY, data.speedZ]
            pass
        elif args == "overheat":
            print("overheat")
            # print data
        elif args == 'cmds':

            marker_array = MarkerArray()

            gate_marker_1 = Marker()
            gate_marker_1.header.frame_id = "vehicle_frame"
            gate_marker_1.header.stamp    = rospy.get_rostime()
            gate_marker_1.ns = "effort"
            gate_marker_1.id = 0
            gate_marker_1.type = 1
            gate_marker_1.action = 0
            gate_marker_1.pose.orientation.w = 1
            gate_marker_1.pose.position.x = data.x*5/2.0
            gate_marker_1.scale.x = data.x*5
            gate_marker_1.scale.y = .04
            gate_marker_1.scale.z = .04
            gate_marker_1.color.r = 1
            gate_marker_1.color.g = 0
            gate_marker_1.color.b = 1
            gate_marker_1.color.a = 1.0
            gate_marker_1.lifetime = rospy.Duration(0)
            marker_array.markers.append(gate_marker_1)

            gate_marker_2 = Marker()
            gate_marker_2.header.frame_id = "vehicle_frame"
            gate_marker_2.header.stamp    = rospy.get_rostime()
            gate_marker_2.ns = "effort"
            gate_marker_2.id = 1
            gate_marker_2.type = 1
            gate_marker_2.action = 0
            gate_marker_2.pose.orientation.w = 1
            gate_marker_2.pose.position.y = data.y*5/2.0
            gate_marker_2.scale.x = .04
            gate_marker_2.scale.y = data.y*5
            gate_marker_2.scale.z = .04
            gate_marker_2.color.r = 1
            gate_marker_2.color.g = 0
            gate_marker_2.color.b = 1
            gate_marker_2.color.a = 1.0
            gate_marker_2.lifetime = rospy.Duration(0)
            marker_array.markers.append(gate_marker_2)

            gate_marker_3 = Marker()
            gate_marker_3.header.frame_id = "vehicle_frame"
            gate_marker_3.header.stamp    = rospy.get_rostime()
            gate_marker_3.ns = "effort"
            gate_marker_3.id = 2
            gate_marker_3.type = 1
            gate_marker_3.action = 0
            gate_marker_3.pose.orientation.w = 1
            gate_marker_3.pose.position.z = (data.z)
            gate_marker_3.scale.x = .04
            gate_marker_3.scale.y = .04
            gate_marker_3.scale.z = (data.z)*2
            gate_marker_3.color.r = 1
            gate_marker_3.color.g = 0
            gate_marker_3.color.b = 1
            gate_marker_3.color.a = 1.0
            gate_marker_3.lifetime = rospy.Duration(0)
            marker_array.markers.append(gate_marker_3)

            gate_marker_4 = Marker()
            gate_marker_4.header.frame_id = "vehicle_frame"
            gate_marker_4.header.stamp    = rospy.get_rostime()
            gate_marker_4.ns = "effort"
            gate_marker_4.id = 3
            gate_marker_4.type = 1
            gate_marker_4.action = 0
            gate_marker_4.pose.orientation.w = 1
            gate_marker_4.pose.position.x = .5
            gate_marker_4.pose.position.y = (data.r)/2
            gate_marker_4.scale.x = .04
            gate_marker_4.scale.y = (data.r)
            gate_marker_4.scale.z = .04
            gate_marker_4.color.r = .5
            gate_marker_4.color.g = .5
            gate_marker_4.color.b = .5
            gate_marker_4.color.a = 1.0
            gate_marker_4.lifetime = rospy.Duration(0)
            marker_array.markers.append(gate_marker_4)
            
            self.vichile_pub.publish(marker_array)


        elif args == 'Image':
            pass

        elif args == 'wp_visual':

            if data.pos.z == 0 and data.hdg == 0:
                return



            marker_array = MarkerArray()
            



            temp_Size = self.gate_size
            temp_normal_pos = [0,0]

                
            
            if(False):
                temp_Size = 2
                temp_normal_pos = [0,0]


                theta = math.pi/4.0
                theta = self.gate_arm_theta
                gate_marker_s = Marker()
                gate_marker_s.header.frame_id = "gate_frame_visual"
                gate_marker_s.header.stamp    = rospy.get_rostime()
                gate_marker_s.ns = "gate_arm"
                gate_marker_s.id = 2
                gate_marker_s.type = 1
                gate_marker_s.action = 0
                gate_marker_s.pose.position.x = -.05
                gate_marker_s.pose.position.y = -.5*.7*math.cos(theta)
                gate_marker_s.pose.position.z = .5*.7*math.sin(theta)

                quat = tf.transformations.quaternion_from_euler(theta, 0, 0)
                gate_marker_s.pose.orientation.x = quat[0]
                gate_marker_s.pose.orientation.y = quat[1]
                gate_marker_s.pose.orientation.z = quat[2]
                gate_marker_s.pose.orientation.w = quat[3]

                gate_marker_s.scale.x = .05
                gate_marker_s.scale.y = .05
                gate_marker_s.scale.z = .7
                gate_marker_s.color.g = 1.0
                gate_marker_s.color.a = 1
                gate_marker_s.lifetime = rospy.Duration(0)            
                marker_array.markers.append(gate_marker_s)


            


            gate_marker_1 = Marker()
            gate_marker_1.header.frame_id = "gate_frame_visual"
            gate_marker_1.header.stamp    = rospy.get_rostime()
            gate_marker_1.ns = "gate_1"
            gate_marker_1.id = self.gate_number
            gate_marker_1.type = 1
            gate_marker_1.action = 0
            gate_marker_1.pose.position.z = temp_Size/2
            gate_marker_1.pose.orientation.w = 1
            gate_marker_1.scale.x = .05
            gate_marker_1.scale.y = temp_Size
            gate_marker_1.scale.z = .05
            gate_marker_1.color.r = 1.0
            gate_marker_1.color.g = .5
            gate_marker_1.color.b = 0.0
            gate_marker_1.color.a = 1.0
            gate_marker_1.lifetime = rospy.Duration(0)
            marker_array.markers.append(gate_marker_1)
            
            gate_marker_2 = Marker()
            gate_marker_2.header.frame_id = "gate_frame_visual"
            gate_marker_2.header.stamp    = rospy.get_rostime()
            gate_marker_2.ns = "gate_2"
            gate_marker_2.id = self.gate_number
            gate_marker_2.type = 1
            gate_marker_2.action = 0
            gate_marker_2.pose.position.y = temp_Size/2
            gate_marker_2.pose.position.z = 0
            gate_marker_2.pose.orientation.w = 1
            gate_marker_2.scale.x = .05
            gate_marker_2.scale.y = .04
            gate_marker_2.scale.z = temp_Size
            gate_marker_2.color.r = 1.0
            gate_marker_2.color.g = .5
            gate_marker_2.color.b = 0.0
            gate_marker_2.color.a = 1.0
            gate_marker_2.lifetime = rospy.Duration(0)
            marker_array.markers.append(gate_marker_2)

            gate_marker_3 = Marker()
            gate_marker_3.header.frame_id = "gate_frame_visual"
            gate_marker_3.header.stamp    = rospy.get_rostime()
            gate_marker_3.ns = "gate_3"
            gate_marker_3.id = self.gate_number
            gate_marker_3.type = 1
            gate_marker_3.action = 0
            gate_marker_3.pose.position.y = -temp_Size/2
            gate_marker_3.pose.orientation.w = 1
            gate_marker_3.scale.x = .05
            gate_marker_3.scale.y = .05
            gate_marker_3.scale.z = temp_Size
            gate_marker_3.color.r = 1.0
            gate_marker_3.color.g = .5
            gate_marker_3.color.b = 0.0
            gate_marker_3.color.a = 1.0
            gate_marker_3.lifetime = rospy.Duration(0)
            marker_array.markers.append(gate_marker_3)

            gate_marker_4 = Marker()
            gate_marker_4.header.frame_id = "gate_frame_visual"
            gate_marker_4.header.stamp    = rospy.get_rostime()
            gate_marker_4.ns = "gate_4"
            gate_marker_4.id = self.gate_number
            gate_marker_4.type = 1
            gate_marker_4.action = 0
            gate_marker_4.pose.position.z = -temp_Size/2
            gate_marker_4.pose.orientation.w = 1
            gate_marker_4.scale.x = .05
            gate_marker_4.scale.y = temp_Size
            gate_marker_4.scale.z = .05
            gate_marker_4.color.r = 1.0
            gate_marker_4.color.g = .5
            gate_marker_4.color.b = 0.0
            gate_marker_4.color.a = 1.0
            gate_marker_4.lifetime = rospy.Duration(0)
            marker_array.markers.append(gate_marker_4)

            gate_marker_n = Marker()
            gate_marker_n.header.frame_id = "gate_frame_visual"
            gate_marker_n.header.stamp    = rospy.get_rostime()
            gate_marker_n.ns = "gate_5n"
            gate_marker_n.id = self.gate_number
            gate_marker_n.type = 1
            gate_marker_n.action = 0
            gate_marker_n.pose.orientation.w = 1
            gate_marker_n.pose.position.y = temp_normal_pos[0]
            gate_marker_n.pose.position.z = temp_normal_pos[1]
            gate_marker_n.scale.x = 3
            gate_marker_n.scale.y = .05
            gate_marker_n.scale.z = .05
            gate_marker_n.color.r = 1.0
            gate_marker_n.color.g = 1.0
            gate_marker_n.color.b = 1.0
            gate_marker_n.color.a = 1.0
            gate_marker_n.lifetime = rospy.Duration(0)
            marker_array.markers.append(gate_marker_n)


            self.gate_pub.publish(marker_array)
            
            # self.tbr.sendTransform((data.t[0],data.t[1],data.t[2]),(quat[0],quat[1],quat[2],quat[3]),rospy.get_rostime(),'gate_frame',"odom")
            quat = tf.transformations.quaternion_from_euler(0, 0, data.hdg)
            self.tbr.sendTransform((data.pos.x,data.pos.y,data.pos.z),(quat[0],quat[1],quat[2],quat[3]),rospy.get_rostime(),'gate_frame_visual',"odom")



        elif args == 'wp_blind':

            if data.pos.z == 0 and data.hdg == 0:
                return
            
            marker_array = MarkerArray()

            
            quat = tf.transformations.quaternion_from_euler(0, 0, data.hdg)
            
            gate_marker_1 = Marker()
            gate_marker_1.header.frame_id = "gate_frame_blind"
            gate_marker_1.header.stamp    = rospy.get_rostime()
            gate_marker_1.ns = "wp_blind_1"
            gate_marker_1.id = self.gate_number
            gate_marker_1.type = 2
            gate_marker_1.action = 0
            gate_marker_1.pose.orientation.w = 1
            gate_marker_1.scale.x = .25
            gate_marker_1.scale.y = .25
            gate_marker_1.scale.z = .25
            gate_marker_1.color.r = .5
            gate_marker_1.color.g = 0
            gate_marker_1.color.b = 1
            gate_marker_1.color.a = 1.0
            gate_marker_1.lifetime = rospy.Duration(0)
            marker_array.markers.append(gate_marker_1)


            gate_marker_4 = Marker()
            gate_marker_4.header.frame_id = "gate_frame_blind"
            gate_marker_4.header.stamp    = rospy.get_rostime()
            gate_marker_4.ns = "wp_blind_2"
            gate_marker_4.id = self.gate_number
            gate_marker_4.type = 1
            gate_marker_4.action = 0
            gate_marker_4.pose.position.x = -.5
            # gate_marker_4.pose.position.y = 0
            gate_marker_4.pose.orientation.w = 1
            gate_marker_4.scale.x = 1
            gate_marker_4.scale.y = .03
            gate_marker_4.scale.z = .03
            gate_marker_4.color.r = 1
            gate_marker_4.color.g = 0
            gate_marker_4.color.b = 1
            gate_marker_4.color.a = 1.0
            gate_marker_4.lifetime = rospy.Duration(0)
            #marker_array.markers.append(gate_marker_4)
            
            # print 'sending blind wp'
            
            self.gate_pub.publish(marker_array)
            
            self.tbr.sendTransform((data.pos.x,data.pos.y,data.pos.z),(quat[0],quat[1],quat[2],quat[3]),rospy.get_rostime(),'gate_frame_blind',"odom")


        elif args == 'wp_look':

            if data.pos.x == 0 and data.pos.y == 0:
                return
            
            marker_array = MarkerArray()
            
            gate_marker_1 = Marker()
            gate_marker_1.header.frame_id = 'wp_look_frame'
            gate_marker_1.header.stamp    = rospy.get_rostime()
            gate_marker_1.ns = "wp_look"
            # gate_marker_1.id = self.gate_number
            gate_marker_1.id = 1
            gate_marker_1.type = 2
            gate_marker_1.action = 0
            gate_marker_1.pose.orientation.w = 1
            gate_marker_1.scale.x = .25
            gate_marker_1.scale.y = .25
            gate_marker_1.scale.z = .75
            gate_marker_1.color.r = 0
            gate_marker_1.color.g = 0
            gate_marker_1.color.b = 1
            gate_marker_1.color.a = 1.0
            gate_marker_1.lifetime = rospy.Duration(0)
            marker_array.markers.append(gate_marker_1)
            
            self.gate_pub.publish(marker_array)            
            self.tbr.sendTransform((data.pos.x,data.pos.y,2),(0,0,0,1),rospy.get_rostime(),'wp_look_frame',"odom")

        elif args == 'wp_current':

            if data.pos.x == 0 and data.pos.y == 0:
                return
            
            marker_array = MarkerArray()
            
            gate_marker_1 = Marker()
            gate_marker_1.header.frame_id = 'wp_current_frame'
            gate_marker_1.header.stamp    = rospy.get_rostime()
            gate_marker_1.ns = "wp_current_1"
            gate_marker_1.id = 1
            gate_marker_1.type = 1
            gate_marker_1.action = 0
            gate_marker_1.pose.orientation.w = 1
            gate_marker_1.scale.x = .05
            gate_marker_1.scale.y = .4
            gate_marker_1.scale.z = .4
            gate_marker_1.color.r = 1
            gate_marker_1.color.g = 0
            gate_marker_1.color.b = 0
            gate_marker_1.color.a = 1.0
            gate_marker_1.lifetime = rospy.Duration(0)
            marker_array.markers.append(gate_marker_1)

            gate_marker_4 = Marker()
            gate_marker_4.header.frame_id = "wp_current_frame"
            gate_marker_4.header.stamp    = rospy.get_rostime()
            gate_marker_4.ns = "wp_current_2"
            gate_marker_4.id = 1
            gate_marker_4.type = 1
            gate_marker_4.action = 0
            gate_marker_4.pose.orientation.w = 1
            gate_marker_4.scale.x = 2
            gate_marker_4.scale.y = .03
            gate_marker_4.scale.z = .03
            gate_marker_4.color.r = 1
            gate_marker_4.color.g = .1
            gate_marker_4.color.b = .1
            gate_marker_4.color.a = 1.0
            gate_marker_4.lifetime = rospy.Duration(0)
            marker_array.markers.append(gate_marker_4)
            
            self.gate_pub.publish(marker_array)  

            quat = tf.transformations.quaternion_from_euler(0, 0, data.hdg)         
            self.tbr.sendTransform((data.pos.x,data.pos.y,data.pos.z),(quat[0],quat[1],quat[2],quat[3]),rospy.get_rostime(),'wp_current_frame',"odom")

def main():
    signal.signal(signal.SIGINT, signal_handler)

    rospy.init_node('bebop_data', anonymous=True)

    
    
    bebop_data()

    rospy.spin() #root.mainloop()


if __name__ == '__main__':
    main()
