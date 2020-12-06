#!/usr/bin/env python

import rospy
import numpy as np 

from duckietown_camera_controller import DuckiebotCamera


if __name__ == "__main__":
    rospy.init_node('duckietown_camera_controller', anonymous=False)
    duckiecam = DuckiebotCamera(show_cam=False, show_processing_ros=False, show_output_ros=False, crop_val=80, window_param=(150,100), warp_param=(140, 190), filter_lower=np.array([80,120,120]), filter_upper = np.array([255,255,255]), vel=0.15, k_d=2.2, k_p_obs = 0.02)
    rospy.on_shutdown(duckiecam.shutdown)
    rospy.spin()
