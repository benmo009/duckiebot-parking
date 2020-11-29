#!/usr/bin/env python
import rospy

# to get commandline arguments
import sys

# because of transformations
import tf

import tf2_ros
import geometry_msgs.msg
import math

if __name__ == '__main__':
    rospy.init_node('my_static_tf2_broadcaster')
    static_transformStamped = geometry_msgs.msg.TransformStamped()

    # Duckiebot and camera transform
    static_transformStamped.header.stamp = rospy.Time.now()
    static_transformStamped.header.frame_id = "duckiebot"
    static_transformStamped.child_frame_id = "camera"

    static_transformStamped.transform.translation.x = 0
    static_transformStamped.transform.translation.y = 0
    static_transformStamped.transform.translation.z = 0.11

    quat = tf.transformations.quaternion_from_euler(
        math.radians(-110),
        math.radians(0),
        math.radians(-90))
    static_transformStamped.transform.rotation.x = quat[0]
    static_transformStamped.transform.rotation.y = quat[1]
    static_transformStamped.transform.rotation.z = quat[2]
    static_transformStamped.transform.rotation.w = quat[3]

    cam_broadcaster = tf2_ros.StaticTransformBroadcaster()
    cam_broadcaster.sendTransform(static_transformStamped)


    # Parking indicator and world transform
    '''
    static_transformStamped.header.frame_id = "world"
    static_transformStamped.child_frame_id = "parking_sign"

    static_transformStamped.transform.translation.x = 2.25
    static_transformStamped.transform.translation.y = 2.1
    static_transformStamped.transform.translation.z = 0

    quat = tf.transformations.quaternion_from_euler(
        math.radians(0),
        math.radians(0),
        math.radians(270))
    static_transformStamped.transform.rotation.x = quat[0]
    static_transformStamped.transform.rotation.y = quat[1]
    static_transformStamped.transform.rotation.z = quat[2]
    static_transformStamped.transform.rotation.w = quat[3]
    parking_broadcaster = tf2_ros.StaticTransformBroadcaster()
    parking_broadcaster.sendTransform(static_transformStamped)
    '''

    '''
    static_transformStamped.header.frame_id = "world"
    static_transformStamped.child_frame_id = "yield_sign"

    static_transformStamped.transform.translation.x = 2.25
    static_transformStamped.transform.translation.y = 2.1
    static_transformStamped.transform.translation.z = 0

    quat = tf.transformations.quaternion_from_euler(
        math.radians(0),
        math.radians(0),
        math.radians(270))
    static_transformStamped.transform.rotation.x = quat[0]
    static_transformStamped.transform.rotation.y = quat[1]
    static_transformStamped.transform.rotation.z = quat[2]
    static_transformStamped.transform.rotation.w = quat[3]
    yield_broadcaster = tf2_ros.StaticTransformBroadcaster()
    yield_broadcaster.sendTransform(static_transformStamped)
    '''


    rospy.spin()
