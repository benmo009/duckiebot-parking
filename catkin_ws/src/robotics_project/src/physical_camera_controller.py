#!/usr/bin/env python

import rospy
import numpy as np 

from duckietown_camera_controller import DuckiebotCamera


if __name__ == "__main__":
    rospy.init_node('duckietown_camera_controller', anonymous=False)


    #show = True
    show = False
    #duckiecam = DuckiebotCamera(show_cam=False, show_processing_ros=show, show_output_ros=show, crop_val=80, window_param=(150,100), warp_param=(140, 190), filter_lower=np.array([120,130,130]), filter_upper = np.array([255,255,255]), vel=0.15, k_d=3.1, k_p_obs = 0.012, k_p = .02)
    #duckiecam = DuckiebotCamera(show_cam=False, show_processing_ros=show, show_output_ros=show, crop_val=80, window_param=(150,100), warp_param=(140, 190), filter_lower=np.array([120,130,130]), filter_upper = np.array([255,255,255]), vel=0.15, k_d=2.9, k_p_obs = 0.012, k_p = .02)
    duckiecam = DuckiebotCamera(show_cam=False, show_processing_ros=show, show_output_ros=show, crop_val=80, window_param=(150,100), warp_param=(140, 190), filter_lower=np.array([120,130,130]), filter_upper = np.array([255,255,255]), vel=0.15, k_d=2.8, k_p_obs = 0.018, k_p = .017)
    rospy.on_shutdown(duckiecam.shutdown)
    rospy.spin()
