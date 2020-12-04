#!/usr/bin/env python

# duckietown_camera_show_binary.py
#
# This file gets image information from the duckietown simulation node and warps
# the image to aerial view then filters out the yellow dotted center line and
# displays the result.

# import time
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
import numpy as np
import cv2
# import sys
# from collections import namedtuple
import math
from math import floor, atan2, pi, cos, sin, sqrt
from cv_bridge import CvBridge, CvBridgeError
# import os
# import time

# Function for cropping and then warping the image
def warp_image(img, left, right):
    # Get the dimensions of the image
    (image_h, image_w, _) = img.shape

    # Set the car position to the center of the window
    car_pos = ((right - left) / 2) + left
    car_pos = np.array([car_pos, image_h-1], dtype=int)

    # Warp the image to get an areal view
    src = np.float32([[0, image_h], [image_w, image_h], [0,0], [image_w, 0]])
    dst = np.float32([[left,image_h], [right, image_h], [0,0], [image_w, 0]])
    M = cv2.getPerspectiveTransform(src, dst)
    Minv = cv2.getPerspectiveTransform(dst, src)  # Use to unwarp the image

    img_warped = cv2.warpPerspective(img, M, (image_w, image_h))

    return img_warped, car_pos, Minv


# Takes an image and returns a filtered binary image
def get_binary_image(img, lower, upper):
    # Convert to HSV color space
    img_hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    # Filter out the specified color
    img_filtered = cv2.inRange(img_hsv, lower, upper)

    # Divide by 255 to convert to binary
    img_bin = img_filtered / 255

    return img_bin


# Function that takes in an image and returns the estimate distance from the center line and angle
def convert_to_binary(img, crop_val=120, new_left=270, new_right=350):
    # Crop the image with specified value
    img = img[crop_val::, :]

    # Warp the image
    img_warped, car_pos, Minv = warp_image(img, new_left, new_right)

    # Filter out the yellow dotted lines
    filter_lower = np.array([20, 100, 150])
    filter_upper = np.array([30, 255, 255])

    # Convert to HSV color space
    img_hsv = cv2.cvtColor(img_warped, cv2.COLOR_BGR2HSV)

    # Filter out the specified color
    img_filtered = cv2.inRange(img_hsv, filter_lower, filter_upper)

    # Divide by 255 to convert to binary
    img_bin = img_filtered / 255

    return img_bin


    
def callback(data):

    bridge=CvBridge()
    #convert images to opencv image
    try:
        cv_image = bridge.imgmsg_to_cv2(data, "8UC3")
    except CvBridgeError as e:
        print(e)
        
    #show image
    cv2.namedWindow("Binary Image")
    if (cv_image is not None):
        img_bin = convert_to_binary(cv_image)
        cv2.imshow("Binary Image",img_bin)

        
    if cv2.waitKey(1)!=-1:
        cv2.destroyAllWindows()

if __name__ == '__main__': 
    rospy.init_node('duckietown_camera_node', anonymous=False)
    sub = rospy.Subscriber("/image_raw",Image,callback)
    pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
    rospy.spin()
