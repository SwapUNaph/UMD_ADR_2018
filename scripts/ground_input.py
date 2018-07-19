#!/usr/bin/env python

#  --- Changelog ---
# Goal:     Display all relevant flight data like position, waypoint, state, video, battery level, wifi status,
# Status:   06/19: There is a gui that displays some information.
#           06/25:

import rospy
import Tkinter as tk
import ttk as ttk
from PIL import ImageTk, Image
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
        rospy.Subscriber("/bebop/cmd_vels", Twist, self.callback, "cmds")
        rospy.Subscriber("/bebop/states/ardrone3/CameraState/Orientation", Ardrone3CameraStateOrientation, self.callback,"cam_orient")
        rospy.Subscriber("/bebop/states/common/CommonState/BatteryStateChanged", CommonCommonStateBatteryStateChanged,self.callback, "battery")
        rospy.Subscriber("/bebop/states/common/CommonState/WifiSignalChanged", CommonCommonStateWifiSignalChanged,self.callback, "wifi")
        rospy.Subscriber("/bebop/states/ardrone3/PilotingState/AltitudeChanged", Ardrone3PilotingStateAltitudeChanged,self.callback, "altitude")
        rospy.Subscriber("/bebop/states/ardrone3/PilotingState/AttitudeChanged", Ardrone3PilotingStateAttitudeChanged,self.callback, "attitude")
        # rospy.Subscriber("/bebop/states/ardrone3/PilotingState/PositionChanged", Ardrone3PilotingStatePositionChanged,  callback, "position") no updates
        rospy.Subscriber("/bebop/states/ardrone3/PilotingState/SpeedChanged", Ardrone3PilotingStateSpeedChanged,self.callback, "speed")
        # rospy.Subscriber("/bebop/states/common/OverHeatState/OverHeatChanged",   OverHeatStateOverHeatChanged,callback, "overheat")
        # rospy.Subscriber("/auto/gate_detection_gate", Image,self.callback, 'Image')
        rospy.Subscriber("/bebop/image_raw", Image,self.callback, 'Image')

        self.bridge = CvBridge()
        self.gate_location = [0,0,0,0,0,0]
        self.pos_data = np.zeros((50,7))
        

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

            np.insert(self.pos_data, 0, [data.pose.pose.position.x, data.pose.pose.position.y, data.pose.pose.position.z, data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z,data.pose.pose.orientation.w],axis=0)
            np.delete(self.pos_data, 49, 0)
            
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
        elif args == 'cmds':
            global value_Z
            global value_R
            global value_X
            global value_Y

            value_X.set(data.linear.x)
            value_Y.set(data.linear.y)
            value_Z.set(data.linear.z)
            value_R.set(data.angular.r)
        elif args == 'Image':
            global img
            global image_space
            rgb = self.bridge.imgmsg_to_cv2(data, desired_encoding=data.encoding)
            # rgb = cv2.cvtColor(rgb, cv2.COLOR_BGR2RGB)
            scalex = 600.0/rgb.shape[1]
            scaley = 400.0/rgb.shape[0]
            res = cv2.resize(rgb,None,fx=scalex, fy=scaley, interpolation = cv2.INTER_CUBIC)
            # cv2.imshow('image',res)
            # if (cv2.waitKey(1) &  0xff == ord('q')):
                # exit()            
            res2 = PIL.Image.fromarray(res)
            img = ImageTk.PhotoImage(res2)
            image_space.create_image(0,0, anchor='nw',image=img)
            
        # print(rospy.get_caller_id() + "\n I heard %s", data)

def main():
    signal.signal(signal.SIGINT, signal_handler)

    rospy.init_node('bebop_data', anonymous=True)

    root = tk.Tk()
    root.title("Bebop GUI")

    # frame1 = ttk.Labelframe(root, text='frame1')
    # frame2 = ttk.Labelframe(root, text='Camera')
    # btn_exit = tk.Button(root, text="Exit", command=root.destroy)
    # frame1.pack()
    # frame2.pack()
    # btn_exit.pack()

    # label_ssid = tk.Label(frame1, text='SSID')
    # label_rssi = tk.Label(frame1, text='RSSI')
    # label_batt = tk.Label(frame1, text='Battery')
    # label_ssid.grid(row=0)
    # label_rssi.grid(row=1)
    # label_batt.grid(row=2)
    # label_ssid_value = tk.Label(frame1)
    # value_rssi = tk.DoubleVar()
    # value_batt = tk.DoubleVar()
    # bar_rssi = ttk.Progressbar(frame1, orient=tk.HORIZONTAL, length=200, mode='determinate', variable=value_rssi)
    # bar_batt = ttk.Progressbar(frame1, orient=tk.HORIZONTAL, length=200, mode='determinate', variable=value_batt)
    # bar_rssi.start(interval=10)
    # bar_batt.start(interval=10)
    # label_ssid_value.grid(row=0, column=1)
    # bar_rssi.grid(row=1, column=1)
    # bar_batt.grid(row=2, column=1)

    # canvas = tk.Canvas(frame2, width=50, height=50)
    # canvas.create_rectangle(0, 0, 50, 50, fill="yellow")
    # canvas.create_line(0, 50-850/21, 50, 50-850/21)
    # canvas.create_line(25, 0, 25, 50)
    # cam_ptr = canvas.create_text(60, 60, text=unichr(9210))
    # canvas.pack()

    state_frame = ttk.Labelframe(root, text='States')
    cmds_frame = ttk.Labelframe(root, text='Commands')
    image_frame = ttk.Labelframe(root,text='Image')
    # plot_frame = ttk.Frame(root)

    state_frame.grid(row=1, column=0)
    cmds_frame.grid(row=2, column=0)
    image_frame.grid(row=0, column=1,rowspan=4)
    # plot_frame.grid(row=0, column=1,rowspan=5)
    

    # States Panel
    global bar_rssi
    global bar_batt
    global value_rssi
    global value_batt
    
    label_rssi = tk.Label(state_frame, text='RSSI')
    label_batt = tk.Label(state_frame, text='Battery')
    label_state = tk.Label(state_frame, text='State')
    
    label_state.grid(row=0)
    label_batt.grid(row=1,columnspan=2)
    label_rssi.grid(row=3,columnspan=2)

    value_rssi = tk.DoubleVar()
    value_batt = tk.DoubleVar()
    value_state = tk.StringVar()

    bar_rssi = ttk.Progressbar(state_frame, orient=tk.HORIZONTAL, length=200, mode='determinate', variable=value_rssi)
    bar_batt = ttk.Progressbar(state_frame, orient=tk.HORIZONTAL, length=200, mode='determinate', variable=value_batt)
    Status_state = tk.Label(state_frame,text=value_state)

    bar_rssi.start(interval=10)
    bar_batt.start(interval=10)
    bar_rssi.grid(row=2, column=0,columnspan=2)
    bar_batt.grid(row=4, column=0,columnspan=2)
    Status_state.grid(row=0,column=1)



    # Commands Panel
    # global canvas
    global value_Z
    global value_R
    global value_X
    global value_Y
    
    value_X = tk.DoubleVar()
    value_Y = tk.DoubleVar()
    value_Z = tk.DoubleVar()
    value_R = tk.DoubleVar()    

    canvas = tk.Canvas(cmds_frame, width=100, height=50)
    canvas.create_rectangle(0, 0, 100, 50, fill="yellow")
    canvas.create_line(0, 25, 100, 25)
    canvas.create_line(50, 0, 50, 50)
    canvas.create_oval(value_X.get()*50-5, value_Y.get()*25-5, value_X.get()*50+5, value_Y.get()*25+5,fill='red')
    bar_throttle = ttk.Progressbar(cmds_frame, orient=tk.HORIZONTAL, length=100, mode='determinate', variable=value_Z)
    bar_yaw = ttk.Progressbar(cmds_frame, orient=tk.HORIZONTAL, length=100, mode='determinate', variable=value_R)
    bar_throttle.start(interval=10)
    bar_yaw.start(interval=10)
    canvas.grid(row=0,column=0,columnspan=2)

    bar_throttle.grid(row=1,column=1)
    bar_yaw.grid(row=2,column=1)
    label_Z = tk.Label(cmds_frame, text='Throttle')
    label_R = tk.Label(cmds_frame, text='Rotation')
    label_Z.grid(row=1,column=0)
    label_R.grid(row=2,column=0)



    # image Panel        
    # global img
    global image_space
    # img = tk.PhotoImage(file='/home/derek/Pictures/dot.png') 
    image_space = tk.Canvas(image_frame, width=600, height=400)  
    # image_space.create_image(100,100, anchor='nw', image=img)
    image_space.pack()
    
    bebop_data()

    root.mainloop()


if __name__ == '__main__':
    main()
