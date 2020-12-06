#!/usr/bin/env python

import math
import numpy as np

import rospy
from std_msgs.msg import Float32
from sensor_msgs.msg import CameraInfo
import tf # Just for quaternion-euler conversions
import tf2_ros

class ParkingDetection:

    def __init__(self):
        rospy.init_node('parking_detection', anonymous=True)

        rospy.on_shutdown(self.shutdown)

        self.cam_sub = rospy.Subscriber('/camera_info', CameraInfo, self.cam_info_callback)
        self.pub_steer = rospy.Publisher('/obstacle_steering', Float32, queue_size=10)

        self.P = None

        # Remember up to 1 second in the past
        tfBuffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(tfBuffer)

        rate = rospy.Rate(2.0)
        while not rospy.is_shutdown():
            '''
            # Get transform from bot to yield sign
            try:
                transform = tfBuffer.lookup_transform('yield_sign', 'duckiebot', rospy.Time(0))
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                rate.sleep()
                continue

            transform_time = transform.header.stamp
            current_time = rospy.Time.now()

            x = transform.transform.translation.x
            y = transform.transform.translation.y
            theta = math.atan2(x, -y)

            dist = math.sqrt(x**2 + y**2)
            print('dist: ' + str(dist))

            print('x: ' + str(x))
            print('y: ' + str(y))
            print('theta: ' + str(math.degrees(theta)))

            if current_time.secs - transform_time.secs <= 1 and dist < 0.3: # and within distance
                #print('Obstacle in view')
                self.pub_steer.publish(theta)
            else:
                #print('Obstacle not in view')
                self.pub_steer.publish(0)
            '''

            try:
                transform = tfBuffer.lookup_transform('yield_sign', 'camera', rospy.Time(0))
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                rate.sleep()
                continue

            transform_time = transform.header.stamp
            current_time = rospy.Time.now()
            
            x = transform.transform.translation.x
            y = transform.transform.translation.y
            z = transform.transform.translation.z
            dist = math.sqrt(x**2 + y**2)
            image_coords = np.matmul(self.P, np.array([x,y,z,1]).reshape(4,1))
            
            x_img = image_coords[0]/image_coords[2]
            y_img = image_coords[1]/image_coords[2]

            print('x: ' + str(x_img))
            print('y: ' + str(y_img))

            if current_time.secs - transform_time.secs < 1 and dist < 0.3: # and within distance
                print('Obstacle in view')
                if (x_img > self.width/2):
                    target = 7*self.width/8
                else:
                    target = self.width/8
                error = x_img - target
                self.pub_steer.publish(error)
                print('error: ' + str(error))
            else:
                print('Obstacle not in view')
                self.pub_steer.publish(0)


            print('---')
            rate.sleep()

    def cam_info_callback(self, msg):
        self.P = np.array(msg.P).reshape(3,4)
        self.width = msg.width

    def shutdown(self):
        if self.pub_steer is not None:
            self.pub_steer.publish(0)
        

if __name__ == "__main__":
    p = ParkingDetection()
