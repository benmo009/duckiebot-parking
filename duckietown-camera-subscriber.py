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

# class CameraSubscriber:
#     def __init__(self):
#         pass

def warp_image(img):
    img = img[120::, :]
    image_H = (img.shape)[0]
    image_W = (img.shape)[1]

    new_Left = 270
    new_Right = 350
    src = np.float32([[0, image_H], [image_W, image_H], [0,0], [image_W, 0]])
    dst = np.float32([[new_Left,image_H], [new_Right, image_H], [0,0], [image_W, 0]])
    M = cv2.getPerspectiveTransform(src, dst)

    warped_img = cv2.warpPerspective(img, M, (image_W, image_H))
    return warped_img
    

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
