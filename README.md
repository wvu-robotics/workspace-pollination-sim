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

### Running SLAM navigation.
Run the following, each in separate terminals (remember to source each):
1. `roslaunch bramblebee_gazebo greenhouse.launch`
This begins the Bramblebee Gazebo model in the greenhouse world. 

2. `roslaunch nav_filter test_online.launch`
This begins SLAM, taking input from Bramblebee's IMU (/imu/data), LIDAR
(velodyne_points), and encoders (/husky_velocity_controller/odom). The output
is /nav_filter/nav_filter/states.

3. `roslaunch bramblebee_navigation move_base_mapless_demo.launch`
This begins MoveIt!.

4. `roslaunch bramblebee_viz view_bramblebee_robot.launch`
This begin RViz, giving visual feedback of the robot's sensors.

### TODO
1. Provide gtsam installation (either prebuilt or source)
2. (?) Provide a small wrapper for starting SLAM navigation so we can work with
less than 6 windows (4 terminals, Gazebo, and RViz).
