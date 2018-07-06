#!/usr/bin/env python

#  --- Changelog ---
# Goal:     Input by bebop camera and stereo camera and state machine and merged_odometry. Depending on state machine, use algorithms to detect gates on the image. Afterwards, position gates on a global map based on the odometry as gate orientation matrices are relative to camera orientation. Output position and orientation of next gate
# Status:   06/19:  Uses cheap stereo camera left video stream to identify gate position and orientation. Does not incorporate any odometry and publishes gate position at (1,0,0) with 3 Hz. Coordainte transformation missing. CV algorithm might need to be improved if there is a performance issue
#           06/25:

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose
from sensor_msgs.msg import Image
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
import math
import matplotlib.pyplot as plt
import time
from bebop_auto.msg import Gate_Detection_Msg

import signal
import sys

camera_matrix = np.array(
        [[608.91474407072610, 0, 318.06264860718505],
         [0, 568.01764400596119, 242.18421070925399],
         [0, 0, 1]], dtype="double"
    )

valid_last_orientation = False

def signal_handler(signal, frame):
    sys.exit(0)


def removeBlob(img):
    #find all your connected components (white blobs in your image)
    nb_components, output, stats, centroids = cv2.connectedComponentsWithStats(img, connectivity=8)
    
    sizes = stats[1:, -1]; nb_components = nb_components - 1

    # minimum size of particles we want to keep (number of pixels)
    
    min_size = 500  

    #your answer image
    image = np.zeros((output.shape))
    #for every component in the image, you keep it only if it's above min_size
    for i in range(0, nb_components):
        if sizes[i] >= min_size:
            image[output == i + 1] = 255

    return image


def image_callback(data):
    global bridge

    
    # convert image msg to matrix
    rgb = bridge.imgmsg_to_cv2(data, desired_encoding=data.encoding)
    image_shape = rgb.shape
    # HSV conversion and frame detection
    hsv = cv2.cvtColor(rgb, cv2.COLOR_BGR2HSV)

    lower_color1 = np.array([150, 160, 30])  # orange 3D
    upper_color1 = np.array([255, 255, 150])  # orange 3D
    lower_color2 = np.array([0, 160, 30])  # orange 3D
    upper_color2 = np.array([20, 255, 150])  # orange 3D

    mask1 = cv2.inRange(hsv, lower_color1, upper_color1)
    mask2 = cv2.inRange(hsv, lower_color2, upper_color2)
    mask = mask1 + mask2


    gray_blur = cv2.GaussianBlur(mask_onimage, (5,5), 0)
    ret, img_postthresh = cv2.threshold(mask_onimage, 50, 255, cv2.THRESH_BINARY)
    mask = cv2.erode(mask, None, iterations=2)
    mask = cv2.dilate(mask, None, iterations=2)

    cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_SIMPLE)
    cnts = cnts[0] if imutils.is_cv2() else cnts[1]
    center = None
 
    # only proceed if at least one contour was found
    if len(cnts) > 0:
        # find the largest contour in the mask, then use
        # it to compute the minimum enclosing circle and
        # centroid
        c = max(cnts, key=cv2.contourArea)
        ((x, y), radius) = cv2.minEnclosingCircle(c)
        M = cv2.moments(c)
        center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
 
        # only proceed if the radius meets a minimum size
        if radius > 10:
            # draw the circle and centroid on the frame,
            # then update the list of tracked points
            cv2.circle(frame, (int(x), int(y)), int(radius),
                (0, 255, 255), 2)
            cv2.circle(frame, center, 5, (0, 0, 255), -1)
 
    
    fov_Y = 90 *3.1415/180
    fov_Z = 70 *3.1415/180

    dist_from_track = 1.5




    width = image_shape[1]
    lateral_error = (center[1]-width/2)/(width*.5)
    lateral_error = lateral_error*dist_from_track*math.tan(fov_Y)

    height = image_shape[0]
    heigth_error = (center[0]-height/2)/(height*.5)
    heigth_error = heigth_error*dist_from_track*math.tan(fov_Z)
    
    cv2.line(rgb, (0,image_shape[1]/2), (image_shape[0],image_shape[1]/2), (0, 255, 255), 10)
    cv2.line(rgb, (image_shape[0]/2,0), (image_shape[0]/2,image_shape[1]), (0, 255, 255), 10)
    cv2.line(rgb, (image_shape[0]/2,image_shape[1]/2), center, (255, 255, 0), 10)

    

    # Display the resulting frame
    cv2.imshow('frame', rgb)


    object_loc = Pose()

    object_loc.position.x = 0.0
    object_loc.position.y = lateral_error
    object_loc.position.z = heigth_error


    global pose_pub
    pose_pub.publish(object_loc)




def main():
    signal.signal(signal.SIGINT, signal_handler)
    rospy.init_node('tracking_test', anonymous=True)

    pose_pub = rospy.Publisher("/auto/path_visual", Pose, queue_size=1)
    rospy.Subscriber("/bebop/raw_image", Image, image_callback)

    global bridge
    bridge = CvBridge()

    rospy.spin()

if __name__ == '__main__':
    main()
