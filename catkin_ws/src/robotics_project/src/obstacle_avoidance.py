#!/usr/bin/env python

import math
import numpy as np
import sys


import rospy
from std_msgs.msg import Float32
from sensor_msgs.msg import CameraInfo
import tf # Just for quaternion-euler conversions
import tf2_ros

class ParkingDetection:

    def __init__(self, use_leds = False):

        rospy.init_node('parking_detection', anonymous=True)

        rospy.on_shutdown(self.shutdown)

        self.cam_sub = rospy.Subscriber('/camera_info', CameraInfo, self.cam_info_callback)
        self.pub_steer = rospy.Publisher('/obstacle_steering', Float32, queue_size=10)

        self.use_leds = use_leds

        self.P = None

        direction = 0

        if self.use_leds:
            # Only import if we are running on raspberry pi
            # error otherwise
            from rgb_led import RGB_LED
            self.led = RGB_LED()
            # rear indicator LEDS
            self.led.setRGB(3, [1,1,1])
            self.led.setRGB(1, [1,1,1])

            # turn on front LEDS
            self.led.setRGB(0, [1,1,1])
            self.led.setRGB(2, [1,1,1])
            self.led.setRGB(4, [1,1,1])

        # Remember up to 1 second in the past
        tfBuffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(tfBuffer)

        rate = rospy.Rate(2.0)
        while not rospy.is_shutdown():

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

            if current_time.secs - transform_time.secs < 2 and dist < 0.2: # and within distance
                print('Obstacle in view')

                if direction == 1 or ((x_img > self.width/2) and direction == 0):
                    direction = 1
                    target = 7*self.width/8
                    #target = self.width
                    error = x_img - target
                    if error > 0:
                        error = 0
                        if self.use_leds:
                            self.led.setRGB(3, [1,1,1])
                    else:
                        if self.use_leds:
                            self.led.setRGB(3, [1,0,0])
                            self.led.setRGB(1, [1,1,1])

                elif direction == -1 or ((x_img < self.width/2) and direction == 0):
                    direction = -1
                    target = self.width/8
                    #target = 0
                    error = x_img - target
                    if error < 0:
                        error = 0
                        if self.use_leds:
                            self.led.setRGB(1, [1,1,1])
                    else:
                        if self.use_leds:
                            self.led.setRGB(1, [1,0,0])
                            self.led.setRGB(3, [1,1,1])
                #error = x_img - target
                self.pub_steer.publish(error)
                print('error: ' + str(error))
            else:
                direction = 0
                if self.use_leds:
                    self.led.setRGB(3, [1,1,1])
                    self.led.setRGB(1, [1,1,1])
                print('Obstacle not in view')
                self.pub_steer.publish(0)


            print('---')
            #rate.sleep()

    def cam_info_callback(self, msg):
        self.P = np.array(msg.P).reshape(3,4)
        self.width = msg.width

    def shutdown(self):
        if self.pub_steer is not None:
            self.pub_steer.publish(0)
        

if __name__ == "__main__":
    use_leds = False
    if (len(sys.argv) > 1):
        if 'led' in sys.argv[1]:
            use_leds = True
    p = ParkingDetection(use_leds=use_leds)
