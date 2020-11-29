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

Run a keyboard teleop node with `$ rosrun teleop_twist_keyboard teleop_twist_keyboard.py`

## Running Teleop Code on Real Robot
Setup the `ROS_MASTER_URI` environment variable on the duckiebot and host computer so that they have the same ros master.

Example: `export ROS_MASTER_URI=http://192.168.1.232:11311/`

Setup the `ROS_IP` environment variable on each computer to the computer's ip.

Example (computer): `export ROS_IP=192.168.1.232`

Example (duckiebot): `export ROS_IP=192.168.1.35`

First start ros with `$ roscore` (on the system set to the Master URI)

Start the keyboard teleop node with `$ rosrun teleop_twist_keyboard teleop_twist_keyboard.py`

Start the twist to motors node on the dukiebot with `$ python twist_to_motors.py` (In the teleop folder)

## Running Parking Autonomous Code on Simulator
Todo