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

Three worlds are available to spawn Bramblebee:
```
roslaunch bramblebee_gazebo empty_world.launch
roslaunch bramblebee_gazebo greenhouse.launch
roslaunch bramblebee_gazebo playpen.launch
```

### TODO
- Add SLAM navigation package(s).