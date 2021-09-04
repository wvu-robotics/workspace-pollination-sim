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

### TODO
1. Provide gtsam installation (either prebuilt or source)
2. (?) Provide a small wrapper for starting SLAM navigation so we can work with
less than 6 windows (4 terminals, Gazebo, and RViz).
