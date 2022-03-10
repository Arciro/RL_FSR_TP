# Technical Project RL/FSR
Development of a control system for a wheeled mobile robot (differential drive robot) used in a logistic environment with ROS-Gazebo

## Getting started
### Requirements 
Make sure that you have installed following useful ROS packages by `sudo apt-get install`:

- `ros-noetic-joy`
- `ros-noetic-gmapping`
- `ros-noetic-amcl`
- `ros-noetic-velocity-controllers`
- `ros-noetic-rqt-multiplot`
- `ros-noetic-zbar-ros`

In fact author had to install them for the developing of the repository.

### Downloading 
1. Download such repository 
2. Unzip it somewhere
3. Cut and paste only `ddr_pkg` in the **/src** folder of your ROS workspace directory

### Compiling
Afterwards compile the code
1. Open a terminal in your ROS workspace folder 
2. Type `catkin_make` command, or if it's necessary ,add also `-DCATKIN_WHITELIST_PACKAGES="ddr_pkg"` command
3. Update the ROS filesystem using the command `rospack profile`

## Running simulation 
Let's start to spawn robot in Gazebo within the scenario and QR-code. So open a new terminal and type in
```shell 
roslaunch ddr_pkg spawn_gazebo.launch
```
in this way, controllers and some parameters will be loaded. Then let's run Odometry custom node, AMCL and open RViz to visualize robot frame, map and markers during the task. In order to do this, type
```shell 
roslaunch ddr_pkg localization_ddr.launch
```
Finally, run planner and controller nodes, along node to read QR-codes, typing
```shell 
roslaunch ddr_pkg task.launch
```

**Note**: if you want to change QR-code, you have to edit png image name in the file qrcode_8cm.material, which you can find at the
path `ddr_pkg/models/QR_Code/material/scripts`
