# WVU Robotics Pollination Simulation

Use this repository to initialize a workspace containing packages used in
Pollination simulations. The goal is requiring no installation steps beyond what
is listed here (assuming a working version of ROS Melodic). At the moment,
the Husky ROS packages are also required to spawn Bramblebee
(`apt install ros-melodic-husky-*`), which will be addressed.

To begin:
```shell
wstool init src pollination.rosinstall
```
`wstool init` simply accesses the git URIs provided in the `pollination.rosinstall` file,
cloning their contents to the `src` directory. **If wstool is installed and
fails here, you likely don't have access to the repositories it tries to clone.
Please notify the `#simulation` Slack channel to be added to the
`wvu-robotics/simulation` team.**

The workspace can then be built:
```shell
catkin build
source devel/setup.bash
```

### Installing tmuxinator for easy launch
Each launch file and ROS node to run the simulation can be run separately in separate terminals, but the simulation can be launched from a single command if tmux and tmuxinator are installed.

```shell
sudo apt install tmux tmuxinator
```

Optional: enable mouse input in tmux for more easily switching between terminal panes.

Edit (or create if it does not yet exist) the file `~/.tmux.conf` and add: `set -g mouse on`

### Running Combined Robot.
The simulation can be launched two different ways. 1) with a single command using tmuxinator, or 2) running each launch file individually

To use option 1), change to the directory where this file is located (the src/ directory inside your workspace) and run:

`tmuxinator start pollination_sim`

To exit the tmux session, press Ctrl+b and then enter the command `kill-session`

To use option 2) run the following, each in separate terminals (remember to source each):
```
roslaunch combined greenhouse.launch //main robot
roslaunch combined combined_viz.launch //rviz visualization and arm move group
roslaunch combined sensor_fusion_sim.launch //SLAM navigation
roslaunch bramblebee_navigation move_base_mapless_demo.launch //move_base
roslaunch manipulation_mapping flower_mapper.launch //classifier nodes
rosrun manipulation_control ee_go_to_pose_action_node //meta move group controller 
roslaunch combined arm_state_machine_sim.launch //arm state machine
rosrun autonomy mission_planning_node
```
Run on a new system to generate lookup table for segmentation service:
```
rosrun manipulation_vision generate_lookup 
```

## Requirements

This software has been tested on Ubuntu 18.04 with ROS Kinetic with OpenCV 3.4.13 as well as OpenCV modules. Please use [these instructions to install OpenCV 3 with the extra modules](https://github.com/wvu-irl/guides-and-resources/wiki/Core-OpenCV-and-Extra-Modules).

This uses submodules, please refer to the [wiki page](https://github.com/wvu-irl/guides-and-resources/wiki/Git-Submodules) on how to work with submodules.

<!-- This software also requires Intel RealSense SDK, please follow these [instructions on how to install](https://github.com/IntelRealSense/librealsense/blob/master/doc/distribution_linux.md) and make sure to install `librealsense2-dkms`, `librealsense2-utils`, and `librealsense2-dev` using the following command:

```
sudo apt-get install -y librealsense2-dkms=1.3.4-0ubuntu1 librealsense2=2.17.1-0~realsense0.372 librealsense2-utils=2.17.1-0~realsense0.372 librealsense2-dev=2.17.1-0~realsense0.372
``` -->

<!-- Intel RealSense working with:
* SDK 1.17.1
* Firmware 5.11.1.0
* intel-ros development branch commit: bc31f2c358037adfa6c3a4e19ccf5845e96ca57a -->

Also, MoveIt must be installed:
```
sudo apt-get install -y ros-melodic-moveit ros-melodic-trac-ik ros-melodic-moveit-visual-tools
```
If ros-melodic-desktop or ros-melodic-ros-base was installed, then pcl_ros and tf2-geometry_msgs most likely need to be separately installed:
```
sudo apt-get install -y ros-melodic-pcl-ros ros-melodic-tf2-geometry-msgs
```

#Pandas
sudo pip install pandas

## Tensorflow       
Install tensorflow (CPU version) via pip:

      sudo apt-get install -y python-pip python-dev
      sudo pip install -U pip
      pip install -U tensorflow==1.4.0 --user
      python -c "import tensorflow as tf; print(tf.__version__)"

## GTSAM
This software requires [GTSAM 4.0.3](https://github.com/borglab/gtsam/releases/tag/4.0.3). 

Make sure GTSAM is built with correct version of Eigen see https://github.com/erik-nelson/blam/issues/48

## PCL
PCL 1.8 is installed with ROS from the `ros-melodic-pcl-ros` package. An issue may be encountered that is an interaction between a conflicting data type between PCL and OpenCV. The best known fix at the moment is described in this [pull request](https://github.com/PointCloudLibrary/pcl/pull/4266). This issue does not occur on some computers, however, so this fix is likely not the best solution. It has to do with some interaction between particular versions of PCL and OpenCV, when both used in the same project.
