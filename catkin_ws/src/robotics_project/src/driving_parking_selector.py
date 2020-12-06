#!/usr/bin/env python

import math

import rospy
from topic_tools.srv import MuxSelect
import tf # Just for quaternion-euler conversions
import tf2_ros

class ParkingDetection:

    def __init__(self):
        rospy.init_node('parking_detection', anonymous=True)

        self.twist_mux_service = None
        rospy.on_shutdown(self.shutdown)

        #self.pub_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        print('waiting for service')
        rospy.wait_for_service('/twist_mux/select')
        print('found service')
        self.twist_mux_service = rospy.ServiceProxy('/twist_mux/select', MuxSelect)

        self.twist_mux_service('twist_lane')

        # Remember up to 1 second in the past
        tfBuffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(tfBuffer)

        rate = rospy.Rate(2.0)
        while not rospy.is_shutdown():
            # Get transform from bot to parking sign
            try:
                transform = tfBuffer.lookup_transform('parking_spot', 'duckiebot', rospy.Time(0))
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                rate.sleep()
                continue

            transform_time = transform.header.stamp
            current_time = rospy.Time.now()

            dist = math.sqrt(transform.transform.translation.x**2 + transform.transform.translation.y**2)
            print('dist: ' + str(dist))
            #rospy.loginfo(msg)

            #if current_time.secs - transform_time.secs <= 1 and dist < 0.6: # and within distance
            #if current_time.secs - transform_time.secs <= 1 and dist < 0.7: # and within distance
            if current_time.secs - transform_time.secs <= 1 and dist < 0.7: # and within distance
                print('Parking spot in view')
                #rospy.loginfo('Parking spot in view')
                self.twist_mux_service('twist_parking')
            else:
                print('Parking spot not in view')
                #rospy.loginfo('Parking spot not in view')
                self.twist_mux_service('twist_lane')


            print('---')
            rate.sleep()

    def shutdown(self):
        if self.twist_mux_service is not None:
            self.twist_mux_service('twist_lane')
        

if __name__ == "__main__":
    p = ParkingDetection()
