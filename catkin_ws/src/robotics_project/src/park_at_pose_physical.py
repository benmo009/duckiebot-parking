#!/usr/bin/env python

import math

import rospy
from geometry_msgs.msg import Twist
import tf # Just for quaternion-euler conversions
import tf2_ros

class DriveToPose:

    def __init__(self):
        rospy.init_node('drive_to_pose', anonymous=True)
        rospy.on_shutdown(self.shutdown_handler)

        #self.kv = 1
        #self.kw = .2

        self.finished = False

        self.pub_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        # Remember up to 1 second in the past
        tfBuffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(tfBuffer)

        rate = rospy.Rate(5.0)
        while not rospy.is_shutdown():
            if self.finished:
                break

            # Get transform from bot to parking sign
            try:
                transform = tfBuffer.lookup_transform('parking_spot', 'duckiebot', rospy.Time(0))
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                rate.sleep()
                continue

            transform_time = transform.header.stamp
            print(transform_time)
            print(transform.transform.translation)
            q = transform.transform.rotation
            eul_rad = tf.transformations.euler_from_quaternion([q.x,q.y,q.z,q.w])
            eul_deg = [math.degrees(angle) for angle in eul_rad]
            print(eul_deg)

            # Drive to position
            '''
            vel_msg = Twist()
            x = transform.transform.translation.x
            y = transform.transform.translation.y
            theta = eul_rad[2]

            vel_msg.linear.x = self.kv * math.sqrt(y**2+x**2)
            if (abs(vel_msg.linear.x) < 0.05):
                vel_msg.linear.x = 0

            print(vel_msg.linear.x)
            
            theta_star = math.atan2(-y, -x)
            vel_msg.angular.z = self.kw * self.min_angle_diff(theta_star, theta)

            self.pub_vel.publish(vel_msg)
            '''

            # Drive to pose
            delta_x = -transform.transform.translation.x
            delta_y = -transform.transform.translation.y
            delta_theta = -eul_rad[2]

            # check if within stopping distance
            c=.25
            dist = math.sqrt(delta_x**2 + delta_y**2 + c*delta_theta**2)
            #dist = math.sqrt(delta_x**2 + delta_y**2)
            print('dist: ' + str(dist))

            vel_msg = Twist()
            #if dist > 0.05:
            if dist > 0.1:
                rho = math.sqrt(delta_x**2 + delta_y**2)

                alpha = self.min_angle_diff(math.atan2(delta_y, delta_x), eul_rad[2])
                beta = self.min_angle_diff(-eul_rad[2], alpha)

                kp = 0.45
                ka = 1
                kb = -0.8

                v = kp*rho
                omega = (ka*alpha+kb*beta)

                print('v: ' + str(v))
                print('omega: ' + str(omega))
                # Minimum linear velocity to avoid stalling
                min_vel = 0
                if (abs(v) < min_vel):
                    v=v/abs(v)*min_vel
                vel_msg.linear.x = v
                vel_msg.angular.z = omega
                print('v_post: ' + str(v))
                print('omega_post: ' + str(omega))
            else:
                vel_msg.linear.x = 0
                vel_msg.angular.z = 0
                self.finished = True
            self.pub_vel.publish(vel_msg)

            print('-----')

            rate.sleep()

    def shutdown_handler(self):
        vel_msg = Twist()
        vel_msg.linear.x = 0
        vel_msg.angular.z = 0
        self.pub_vel.publish(vel_msg)

    def min_angle_diff(self, theta_1, theta_2):
        delta_theta = theta_1-theta_2
        tau=2*math.pi

        return min(delta_theta, delta_theta + tau, delta_theta - tau, key=abs)

if __name__ == "__main__":
    p = DriveToPose()
