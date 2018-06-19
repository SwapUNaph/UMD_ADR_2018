#!/usr/bin/env python

import cv2
import numpy as np

select = 6
# Read Image
#2D image points. If you change the image, you need to change vector
if select==1:
	im = cv2.imread("i1.jpg");
	image_points = np.array([
                            (1467,314),
                            (1773,199),
                            (1460,654),
                            (1763,562)
                        ], dtype="double")
elif select==2:
	im = cv2.imread("i2.jpg");
	image_points = np.array([
                            (611, 2886),
                            (1223, 2809),
                            (437, 3556),
                            (1115, 3520)
                        ], dtype="double")
elif select==3:
	im = cv2.imread("i3.jpg");
	image_points = np.array([
                            (1401, 430),
                            (1827, 433),
                            (1388, 825),
                            (1812, 827)
                        ], dtype="double")
elif select==4:
	im = cv2.imread("i4.jpg");
	image_points = np.array([
                            (1146,1971),
                            (1516,1959),
                            (1143,2337),
                            (1513,2349)
                        ], dtype="double")
elif select==5:
	im = cv2.imread("i5.jpg");
	image_points = np.array([
                            (746,2253),
                            (1197,2229),
                            (712,2655),
                            (1185,2627)
                        ], dtype="double")
elif select==6:
	im = cv2.imread("i6.jpg");
	image_points = np.array([
                            (2263,279),
                            (2803,135),
                            (2275,765),
                            (2814,652)
                        ], dtype="double")


size = im.shape

# draw rectangle
pts = np.array([image_points[0],image_points[1],image_points[3],image_points[2]], np.int32)
pts = pts.reshape((-1,1,2))
cv2.polylines(im,[pts],True,(255,255,255),10)

# 3D model points.
model_points = np.array([
                            (0.0, 0.0, 0.0),
                            (.163, 0.0, 0.0),
                            (0.0, .148, 0.0),
                            (.163, .148, 0.0)
                        ])

# Camera internals
focal_length = 4160*4/4.8   #4mm, 3120x4160. sensor: 4.8x3.6mm

focal_length = size[0]*4/4.8
center = (size[1]/2, size[0]/2)
camera_matrix = np.array(
                         [[focal_length, 0, center[0]],
                         [0, focal_length, center[1]],
                         [0, 0, 1]], dtype = "double"
                         )

print "Camera Matrix :\n {0}".format(camera_matrix);

dist_coeffs = np.zeros((4,1)) # Assuming no lens distortion
(success, rotation_vector, translation_vector) = cv2.solvePnP(model_points, image_points, camera_matrix, dist_coeffs, flags=cv2.SOLVEPNP_ITERATIVE)

print "Rotation Vector:\n {0}".format(rotation_vector)
print "Translation Vector:\n {0}".format(translation_vector)

# Project a 3D point (0, 0, 1000.0) onto the image plane.
# We use this to draw a line sticking out of the nose

(nose_end_point2D_1, jacobian_1) = cv2.projectPoints(np.array([(.0815, .074, 0)]), rotation_vector, translation_vector, camera_matrix, dist_coeffs)
(nose_end_point2D_2, jacobian_2) = cv2.projectPoints(np.array([(.0815, .074, -.155)]), rotation_vector, translation_vector, camera_matrix, dist_coeffs)

for p in image_points:
    cv2.circle(im, (int(p[0]), int(p[1])), 10, (0,0,255), -1)

p1 = ( int(nose_end_point2D_1[0][0][0]), int(nose_end_point2D_1[0][0][1]))
p2 = ( int(nose_end_point2D_2[0][0][0]), int(nose_end_point2D_2[0][0][1]))

cv2.line(im, p1, p2, (255,255,255), 10)

# Display image
cv2.namedWindow('image',cv2.WINDOW_NORMAL)
cv2.resizeWindow('image', 800,800)
cv2.imshow('image', im);
cv2.waitKey(0);