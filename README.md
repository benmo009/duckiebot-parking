# duckiebot-parking

Duckiebot Needs to Park

Use these repositories:  
[gym-duckietown](https://github.com/duckietown/gym-duckietown)  
[Gym-Duckietown-DuckiebotSimulation-RR-ROS-wrappers](https://github.com/burakaksoy/Gym-Duckietown-DuckiebotSimulation-RR-ROS-wrappers)

Follow the installation instructions in the gym-duckietown repository

Setup the custom map by copying the files in `sim_files/` to their respective folders in `gym-duckietown/gym-duckietown`.


Setup the catkin workspace in the repository. See ROS [workspace documentation](http://wiki.ros.org/catkin/Tutorials/create_a_workspace)
## Running Teleop Code On Simulator
First start ros with `$ roscore`

Run just the sim with `$ rosrun robotics_project sim.py`

(Note: You may have to comment out line 451 in `gym-duckietown/gym-duckietown/simulator.py` : `logger.info('using user tile start: %s' % self.user_tile_start)`

Run a keyboard teleop node with `$ rosrun teleop_twist_keyboard teleop_twist_keyboard.py`

If you would like to run the robot with all nodes set up for Apriltag Decoding, run `$ roslaunch robotics_project run.launch` and `$ roslaunch robotics_project apriltag.launch`

## Running Teleop Code on Real Robot
Setup the `ROS_MASTER_URI` environment variable on the duckiebot and host computer so that they have the same ros master.

Example: `export ROS_MASTER_URI=http://192.168.1.232:11311/`

Setup the `ROS_IP` environment variable on each computer to the computer's ip.

Example (computer): `export ROS_IP=192.168.1.232`

Example (duckiebot): `export ROS_IP=192.168.1.35`

First start ros with `$ roscore` (on the system set to the Master URI)

Start the keyboard teleop node with `$ rosrun teleop_twist_keyboard teleop_twist_keyboard.py`

Start the twist to motors node on the dukiebot with `$ python twist_to_motors.py` (In the teleop folder)

## Running Lane Following and Parking Autonomous Code on Simulator

First start ros with `$ roscore`

Make sure to setup the sim with the custom map following the instructions above

Run apriltags_ros with `$ roslaunch robotics_project apriltag_sim.launch`

This just launches the apriltag node, but was kept seperate to be able to stop and start it to change settings without needing to restart everything else.

Start the sim and supporting nodes with `$ roslaunch robotics_project run_sim.launch`. This launches the folling nodes:

- sim.py: Simulator on custom parking map
- static_tf2_broadcaster.py: Broadcasts relative transform between sign+parking spot, and between camera+duckiebot
- image_proc: Uses camera parameters to rectify the image
- rqt_gui: Can be used to view the image streams, set it to '/tag_detections_image' to view the apriltag output
- rviz: Opens rviz with the transform tree showing
- mux: Selects between the parking twist and the lane following tesit
- driving_parking_selector.py: Checks if the parking tag is close enough to park. Sends commands to the mux to select either lane following or parking
- park_at_pose.py: Sends twist commands to park in front of the apriltag, based on transfroms from statc_tf2_broadcaster.py
- duckietown_camera_controller.py: Sends twist commands to follow the lane



## Visual Lane Following

Use the Jupyter Notebook `find_lane.ipynb` to go through the process of estimating the car's pose relative to the center lane line for one image.

In one terminal, start ROS with `$ roscore`  

Open another terminal and start the simulation node `$ python duckietown-sim-node.py`

Open a third terminal and start the camera controler node `$ python duckietown_camera_controller.py`

Optionally, run a node that shows the warped and filtered perspective with `$ python duckietown_camera_show_binary.py`

Video demo in `visual_lane_following_demo.mp4`

TODO:  

- adjust gains for better control
- Build using ROS catkin system so that it can be launched using `roslaunch`  
