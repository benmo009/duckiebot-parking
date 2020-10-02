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

if __name__ == "__main__":
    try:
        rospy.init_node('input_velocity')
        pub = rospy.Publisher("/cmd_vel",Twist)

        velocity = Twist()
        velocity.linear.x = 1;


        while(1):
            pub.publish(velocity)
            rospy.loginfo("Published velocity")
    except rospy.ROSInterruptException:
        pass