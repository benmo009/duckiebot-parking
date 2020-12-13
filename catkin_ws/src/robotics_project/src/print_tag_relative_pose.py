#!/usr/bin/env python

import math

import rospy
import tf # Just for quaternion-euler conversions
import tf2_ros
import numpy as np

class DriveToPose:

    def __init__(self):
        rospy.init_node('print_tag_pose', anonymous=True)

        # Remember up to 1 second in the past
        tfBuffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(tfBuffer)

        rate = rospy.Rate(5.0)
        while not rospy.is_shutdown():
            raw_input("Press Enter to Take a Reading")
            # Take 10 samples
            x = []
            y = []
            theta = []
            for i in range(10):
                # Get transform from bot to parking sign
                try:
                    transform = tfBuffer.lookup_transform('duckiebot', 'parking_spot', rospy.Time(0))
                except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                    rate.sleep()
                    continue

                transform_time = transform.header.stamp
                #print(transform_time)
                #print(transform.transform.translation)
                q = transform.transform.rotation
                eul_rad = tf.transformations.euler_from_quaternion([q.x,q.y,q.z,q.w])
                eul_deg = [math.degrees(angle) for angle in eul_rad]
                #print(eul_deg)

                #print('-----')
                x.append(transform.transform.translation.x)
                y.append(transform.transform.translation.y)
                theta.append(eul_deg[2])

                rate.sleep()

            print('x: avg: %.3f, stdev: %.5f' % (np.average(x), np.std(x)))
            print('y: avg: %.3f, stdev: %.5f' % (np.average(y), np.std(y)))
            print('theta: avg: %.1f, stdev: %.5f' % (np.average(theta), np.std(theta)))
            #print('x: avg: ' + "{:.2f}".format(str(np.average(x))) + ', stdev: ' + str(np.std(x)))
            #print('y: avg: ' + str(np.average(y)) + ', stdev: ' + str(np.std(y)))
            #print('theta: avg: ' + str(np.average(theta)) + ', stdev: ' + str(np.std(theta)))


if __name__ == "__main__":
    p = DriveToPose()
