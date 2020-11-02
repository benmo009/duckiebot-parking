#!/usr/bin/env python
import time
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
import numpy as np
import cv2
import sys
from collections import namedtuple
import math
from math import floor, atan2, pi, cos, sin, sqrt
from cv_bridge import CvBridge, CvBridgeError
import os
import time

# Comverts image to aerial view and then filters out the lane lines
def warp_image(img):
    img = img[120::, :]
    image_H = (img.shape)[0]
    image_W = (img.shape)[1]

    new_Left = 270
    new_Right = 350
    src = np.float32([[0, image_H], [image_W, image_H], [0,0], [image_W, 0]])
    dst = np.float32([[new_Left,image_H], [new_Right, image_H], [0,0], [image_W, 0]])
    M = cv2.getPerspectiveTransform(src, dst)

    img_warped = cv2.warpPerspective(img, M, (image_W, image_H))

    # Transform image to HSV color space
    img_hsv = cv2.cvtColor(img_warped, cv2.COLOR_BGR2HSV)

    # Filter out everything but yellow (dotted lines) and white (side lines)
    yellow_lower = np.array([20, 75, 100])
    yellow_upper = np.array([30, 255, 255])
    white_lower = np.array([0,0,1])
    white_upper = np.array([120,5,255])

    img_yellow = cv2.inRange(img_hsv, yellow_lower, yellow_upper)
    img_white = cv2.inRange(img_hsv, white_lower, white_upper)
    combined_filtered = img_yellow + img_white
    img_filtered = cv2.bitwise_and(img_warped, img_warped, mask=combined_filtered)
    
    return img_filtered
    

def callback(data):

    bridge=CvBridge()
    #convert images to opencv image
    try:
        cv_image = bridge.imgmsg_to_cv2(data, "8UC3")
    except CvBridgeError as e:
        print(e)
        
    #show image
    cv2.namedWindow("Image")
    if (cv_image is not None):
        cv_image = warp_image(cv_image)
        cv2.imshow("Image",cv_image)

        # # Save the image
        # filename = time.strftime("%Y%m%d-%H%M%S")
        # cv2.imwrite('duckietown_images/{:s}.png'.format(filename), cv_image)

        
    if cv2.waitKey(1)!=-1:     #Burak, needs to modify this line to work on your computer, THANKS!
        cv2.destroyAllWindows()

if __name__ == '__main__': 
    rospy.init_node('duckietown_camera_node', anonymous=False)
    sub = rospy.Subscriber("/image_raw",Image,callback)
    rospy.spin()
