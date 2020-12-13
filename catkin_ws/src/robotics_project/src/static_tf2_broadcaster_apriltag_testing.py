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

    broadcaster = tf2_ros.StaticTransformBroadcaster()

    duckiebot_camera_transform = geometry_msgs.msg.TransformStamped()

    # Duckiebot and camera transform
    duckiebot_camera_transform.header.stamp = rospy.Time.now()
    duckiebot_camera_transform.header.frame_id = "duckiebot"
    duckiebot_camera_transform.child_frame_id = "camera"

    duckiebot_camera_transform.transform.translation.x = 0
    duckiebot_camera_transform.transform.translation.y = 0
    duckiebot_camera_transform.transform.translation.z = 0.108
    #duckiebot_camera_transform.transform.translation.z = 0

    quat = tf.transformations.quaternion_from_euler(
        math.radians(-105),
        math.radians(0),
        math.radians(-90))
    duckiebot_camera_transform.transform.rotation.x = quat[0]
    duckiebot_camera_transform.transform.rotation.y = quat[1]
    duckiebot_camera_transform.transform.rotation.z = quat[2]
    duckiebot_camera_transform.transform.rotation.w = quat[3]

    # Parking indicator and spot
    parking_transform = geometry_msgs.msg.TransformStamped()
    #parking_transform.header.frame_id = "parking_sign"
    parking_transform.header.stamp = rospy.Time.now()
    parking_transform.header.frame_id = "parking_sign"
    parking_transform.child_frame_id = "parking_spot"

    parking_transform.transform.translation.x = 0
    parking_transform.transform.translation.y = -.04
    #parking_transform.transform.translation.z = .07
    parking_transform.transform.translation.z = 0

    quat = tf.transformations.quaternion_from_euler(
        math.radians(-90),
        math.radians(90),
        math.radians(0))
    parking_transform.transform.rotation.x = quat[0]
    parking_transform.transform.rotation.y = quat[1]
    parking_transform.transform.rotation.z = quat[2]
    parking_transform.transform.rotation.w = quat[3]


    broadcaster.sendTransform([duckiebot_camera_transform, parking_transform])


    rospy.spin()
