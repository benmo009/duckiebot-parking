#!/usr/bin/env python
import rospy
import io

import cv2
from cv_bridge import CvBridge, CvBridgeError

from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image, CameraInfo

###
import sys
import argparse
# import pyglet
# from pyglet.window import key
import numpy as np
import gym
import gym_duckietown
from gym_duckietown.envs import DuckietownEnv
from gym_duckietown.wrappers import UndistortWrapper
###

class DuckiebotSim(object):
    def __init__(self):
        # Initialize Robot Simulation
        self.env = DuckietownEnv( seed = 1, map_name = 'loop_sid', draw_curve = False, draw_bbox = False, distortion = True, domain_rand = False, camera_width=640, camera_height=480, user_tile_start=(1,3))
        self.env.reset()
        self.env.render()
        # self.action = np.array([0.44, 0.0])
        self.action = np.array([0.0, 0.0])
        
        # For image to ros
        self.bridge = CvBridge()
        
        # Initialize ROS Node
        self.node_name = rospy.get_name()
        rospy.loginfo("[%s] Initializing......" %(self.node_name))
        
        # Setup parameters
        self.framerate = self.setupParam("~framerate",self.env.unwrapped.frame_rate)
        
        # Setup Publisher
        #self.pub_img= rospy.Publisher("cv_camera/image_raw",Image,queue_size=1)
        #self.pub_cam = rospy.Publisher("cv_camera/camera_info", CameraInfo, queue_size=1)
        self.pub_img= rospy.Publisher("image_raw",Image,queue_size=1)
        self.pub_cam = rospy.Publisher("camera_info", CameraInfo, queue_size=1)
        self.has_published = False
        
        # Setup Subscriber
        self.sub_cmd = rospy.Subscriber("/cmd_vel", Twist, self.cbCmd, queue_size=1)
        
        
        # Setup timer
        self.timer_img_low = rospy.Timer(rospy.Duration.from_sec(1.0/self.framerate),self.cbTimer)
        #self.timer_img_low = rospy.Timer(rospy.Duration.from_sec(1.0/10.0),self.cbTimer)
        rospy.loginfo("[%s] Initialized." %(self.node_name))
    
    def setupParam(self,param_name,default_value):
        value = rospy.get_param(param_name,default_value)
        rospy.set_param(param_name,value) #Write to parameter server for transparancy
        rospy.loginfo("[%s] %s = %s " %(self.node_name,param_name,value))
        return value
        
    def cbTimer(self,event):
        if not rospy.is_shutdown():
            self.grabAndPublish(self.pub_img)
        
    def grabAndPublish(self,publisher):
        # Grab image from simulation and apply the action
        obs, reward, done, info = self.env.step(self.action)
        #rospy.loginfo('[%s] step_count = %s, reward=%.3f' % (self.node_name, self.env.unwrapped.step_count, reward))
        
        image = cv2.cvtColor(obs, cv2.COLOR_BGR2RGB) # Correct color for cv2
        """
        ##show image
        cv2.namedWindow("Image")
        if (not image is None):
            cv2.imshow("Image",image)
        if cv2.waitKey(1)!=-1:     #Burak, needs to modify this line to work on your computer, THANKS!
            cv2.destroyAllWindows()
        """
        # Publish raw image
        image_msg = self.bridge.cv2_to_imgmsg(image, "bgr8")
                
        image_msg.header.stamp = rospy.Time.now()
        # Publish 
        publisher.publish(image_msg)

        cam_msg = CameraInfo()
        cam_msg.height = 480
        cam_msg.width= 640
        #cam_msg.height = 240
        #cam_msg.width= 320
        cam_msg.header.stamp = image_msg.header.stamp
        cam_msg.header.frame_id='cam'

        cam_msg.distortion_model = 'plumb_bob'
        cam_msg.D = [-0.2, 0.0305, 0.0005859930422629722,-0.0006697840226199427,0]
        cam_msg.K = [
            305.5718893575089, 0, 303.0797142544728,
            0, 308.8338858195428, 231.8845403702499,
            0, 0, 1,
        ]
        cam_msg.R = [1.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,1.0]
        cam_msg.P = [
            220.2460277141687, 0, 301.8668918355899, 0,
            0, 238.6758484095299, 227.0880056118307, 0,
            0, 0, 1, 0,
        ]


        self.pub_cam.publish(cam_msg)


        if not self.has_published:
            rospy.loginfo("[%s] Published the first image." %(self.node_name))
            self.has_published = True
        
        #if done and (self.env.unwrapped.step_count != 1500):
            #rospy.logwarn("[%s] DONE! RESETTING." %(self.node_name))
            #self.env.reset()
            
        self.env.render()
        
    def cbCmd(self,cmd_msg):
        linear_vel = cmd_msg.linear.x # Forward Velocity [-1 1]
        angular_vel = cmd_msg.angular.z # Steering angle [-1 1]
        self.action = np.array([linear_vel, angular_vel])
        
    def onShutdown(self):
        rospy.loginfo("[%s] Shutdown." %(self.node_name))
        self.env.close()
        
if __name__ == '__main__': 
    rospy.init_node('duckiebot_sim',anonymous=False)
    duckiebot_sim = DuckiebotSim()
    rospy.on_shutdown(duckiebot_sim.onShutdown)
    rospy.spin()
