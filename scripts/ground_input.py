#!/usr/bin/env python

#  --- Changelog ---
# Goal:     Display all relevant flight data like position, waypoint, state, video, battery level, wifi status,
# Status:   06/19: There is a gui that displays some information.
#           06/25:

import rospy
import Tkinter as tk
import ttk as ttk
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty
from nav_msgs.msg import Odometry
from bebop_msgs.msg import Ardrone3CameraStateOrientation
from bebop_msgs.msg import CommonCommonStateBatteryStateChanged
from bebop_msgs.msg import CommonCommonStateWifiSignalChanged
from bebop_msgs.msg import Ardrone3PilotingStateAltitudeChanged
from bebop_msgs.msg import Ardrone3PilotingStateAttitudeChanged
from bebop_msgs.msg import Ardrone3PilotingStateSpeedChanged
#from bebop_msgs.msg import OverHeatStateOverHeatChanged

import signal
import sys

def signal_handler(signal, frame):
    sys.exit(0)

class bebop_data:
    def __init__(self):
        rospy.Subscriber("/bebop/camera_control", Twist, self.callback, "cam_ctrl")
        rospy.Subscriber("/bebop/reset", Empty, self.callback, "reset")
        rospy.Subscriber("/bebop/odom", Odometry, self.callback, "odom")
        rospy.Subscriber("/bebop/states/ardrone3/CameraState/Orientation", Ardrone3CameraStateOrientation, self.callback,
                         "cam_orient")
        rospy.Subscriber("/bebop/states/common/CommonState/BatteryStateChanged", CommonCommonStateBatteryStateChanged,
                         self.callback, "battery")
        rospy.Subscriber("/bebop/states/common/CommonState/WifiSignalChanged", CommonCommonStateWifiSignalChanged,
                         self.callback, "wifi")
        rospy.Subscriber("/bebop/states/ardrone3/PilotingState/AltitudeChanged", Ardrone3PilotingStateAltitudeChanged,
                         self.callback, "altitude")
        rospy.Subscriber("/bebop/states/ardrone3/PilotingState/AttitudeChanged", Ardrone3PilotingStateAttitudeChanged,
                         self.callback, "attitude")
        # rospy.Subscriber("/bebop/states/ardrone3/PilotingState/PositionChanged", Ardrone3PilotingStatePositionChanged,  callback, "position") no updates
        rospy.Subscriber("/bebop/states/ardrone3/PilotingState/SpeedChanged", Ardrone3PilotingStateSpeedChanged,
                         self.callback, "speed")

        #    rospy.Subscriber("/bebop/states/common/OverHeatState/OverHeatChanged",   OverHeatStateOverHeatChanged,          callback, "overheat")

        # ~states/enable_pilotingstate_flyingstatechanged parameter to true
        # enable the publication of flying state changes to topic states/ARDrone3/PilotingState/FlyingStateChanged

    def callback(self,data,args):
        # rospy.loginfo(rospy.get_caller_id() + "\nI heard %s", data)
        if args == "cam_ctrl":
            # canvas.itemconfig(cam_text)
            # canvas.coords(cam_ptr, 25 * (data.angular.z / 60 + 1), 25 * (-data.angular.y / 90 + 1))
            pass
        elif args == "reset":
            # print("reset pressed")
            pass
        elif args == "odom":
            print("odometry: ...")
            print("position")
            print [data.pose.pose.position.x, data.pose.pose.position.y, data.pose.pose.position.z]
            print [data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z,
                   data.pose.pose.orientation.w]
            print [data.twist.twist.linear.x, data.twist.twist.linear.y, data.twist.twist.linear.z]
            print [data.twist.twist.angular.x, data.twist.twist.angular.y, data.twist.twist.angular.z]

            print("---")
        elif args == "cam_orient":
            canvas.coords(cam_ptr, 5.0 / 7.0 * data.pan + 25, -10.0 / 21.0 * data.tilt + 200 / 21)
        elif args == "battery":
            # canvas.itemconfig(bar_batt, mode=determinate)
            bar_batt.stop()
            value_batt.set(data.percent)
        elif args == "wifi":
            # canvas.itemconfig(bar_rssi, mode=determinate)
            bar_rssi.stop()
            value_rssi.set(data.rssi * 2 + 160)
        elif args == "altitude":
            print("altitude")
            print data.altitude
        elif args == "attitude":
            print("attitude")
            print [data.roll, data.pitch, data.yaw]
        elif args == "speed":
            print("speed")
            print [data.speedX, data.speedY, data.speedZ]
        elif args == "overheat":
            print("overheat")
            # print data

        # print(rospy.get_caller_id() + "\n I heard %s", data)

def main():
    signal.signal(signal.SIGINT, signal_handler)

    root = tk.Tk()
    root.title("Bebop GUI")

    frame1 = ttk.Labelframe(root, text='frame1')
    frame2 = ttk.Labelframe(root, text='Camera')
    btn_exit = tk.Button(root, text="Exit", command=root.destroy)
    frame1.pack()
    frame2.pack()
    btn_exit.pack()

    label_ssid = tk.Label(frame1, text='SSID')
    label_rssi = tk.Label(frame1, text='RSSI')
    label_batt = tk.Label(frame1, text='Battery')
    label_ssid.grid(row=0)
    label_rssi.grid(row=1)
    label_batt.grid(row=2)
    label_ssid_value = tk.Label(frame1)
    value_rssi = tk.DoubleVar()
    value_batt = tk.DoubleVar()
    bar_rssi = ttk.Progressbar(frame1, orient=tk.HORIZONTAL, length=200, mode='determinate', variable=value_rssi)
    bar_batt = ttk.Progressbar(frame1, orient=tk.HORIZONTAL, length=200, mode='determinate', variable=value_batt)
    bar_rssi.start(interval=10)
    bar_batt.start(interval=10)
    label_ssid_value.grid(row=0, column=1)
    bar_rssi.grid(row=1, column=1)
    bar_batt.grid(row=2, column=1)

    canvas = tk.Canvas(frame2, width=50, height=50)
    canvas.create_rectangle(0, 0, 50, 50, fill="yellow")
    canvas.create_line(0, 50-850/21, 50, 50-850/21)
    canvas.create_line(25, 0, 25, 50)
    cam_ptr = canvas.create_text(60, 60, text=unichr(9210))
    canvas.pack()

    rospy.init_node('bebop_data', anonymous=True)
    bebop_data()

    root.mainloop()


if __name__ == '__main__':
    main()
