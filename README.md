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

### Running Combined Robot.
Run the following, each in separate terminals (remember to source each):
```
roslaunch combined greenhouse.launch //main robot
roslaunch combined combined_viz.launch //rviz visualization and arm move group
roslaunch combined nav_filter.launch //SLAM navigation
roslaunch bramblebee_navigation move_base_mapless.launch //move_base
roslaunch manipulation_mapping flower_mapper.launch //classifier nodes
rosrun manipulation_control ee_go_to_pose //meta move group controller for arm
rosrun manipulation_mapping pre_pose_mapping.py //prepose mapping sequence
rosrun manipulation_state_machine planning_ga_ros.py
rosrun manipulation_state_machine state_machine_pollinating.py
rosrun autonomy mission_planning_node
```

### TODO
1. Provide gtsam installation (either prebuilt or source)
2. (?) Provide a small wrapper for starting SLAM navigation so we can work with
less than 6 windows (4 terminals, Gazebo, and RViz).
