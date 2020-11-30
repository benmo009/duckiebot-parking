import time

import rospy
from geometry_msgs.msg import Twist

def stop(pub):
    vel = Twist()
    pub.publish(vel)

def linear_test(pub):
    vel = Twist()
    vel.linear.x = 0.5
    pub.publish(vel)

    time.sleep(1)

    stop(pub)

def angular_test(pub):
    vel = Twist()
    vel.angular.z = 3.14*2
    pub.publish(vel)

    time.sleep(1)

    stop(pub)

def linear_angular_test(pub):
    vel = Twist()
    vel.linear.x = 0.5
    vel.angular.z = 3.14
    pub.publish(vel)

    time.sleep(2)

    stop(pub)

if __name__ == '__main__':
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rospy.init_node('open_loop_drive', anonymous=False)

    #linear_test(pub)
    angular_test(pub)
    #linear_angular_test(pub)
