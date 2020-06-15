# WVU Robotics Pollination Simulation

Use this repository to pull in all packages used in the Pollination project's
simulation environment. A script is provided to prepare it as a catkin
workspace:
```shell
. init-src
```

Three worlds are available to spawn Bramblebee:
```
roslaunch bramblebee_gazebo empty_world.launch
roslaunch bramblebee_gazebo greenhouse.launch
roslaunch bramblebee_gazebo playpen.launch
```