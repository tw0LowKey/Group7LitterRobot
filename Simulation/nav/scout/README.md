### Download required packages
 Download and install gazebo-ros function package, gazebo-ros is the communication interface between gazebo and ROS, and connect the ROS and Gazebo
```bash
    sudo apt-get install ros-humble-gazebo-*
```
Download and install joint-state-publisher-gui joint-state-publisher package.This package is used to visualize the joint control.
```bash
    sudo apt-get install ros-humble-joint-state-publisher ros-humble-joint-state-publisher-gui
```
​ Download and install ackermann-steering controller; The ackermann-steering controller is a gazebo plugin for controlling the car
```bash
    sudo apt-get install ros-humble-ackermann-steering-controller
```
Download and install Control-related dependencies and feature packs; control is used to define the type of model joints
```bash
    sudo apt-get install ros-humble-control-*
```

Download and install rqt-robot-steering plug-in, rqt_robot_steering is a ROS tool closely related to robot motion control, it can send the control command of robot linear motion and steering motion, and the robot motion can be easily controlled through the sliding bar

```bash
    sudo apt-get install ros-humble-rqt-robot-steering 
```

### Download rosdep
```bash
sudo apt update
sudo apt install python3-rosdep2
```
initalise 
```bash
sudo rosdep init
rosdep update
```


### Usage for the Scout Mini
Building the package: 
```bash
    colcon build
```
Source the built packages:
```bash
    . install/setup.bash
```
Launch the simulation environment:
```bash
    ros2 launch scout_gazebo_sim scout_mini_empty_world.launch.py
```
### Parameters of <i>scout_mini_empty_world.launch.py</i>:
- <b>use_rviz</b>: if you want to launch rviz2 or not, default <i>true</i>.
- <b>rviz_config_file</b>: name of the configuration file for RVIz2, defautl <i>scout_mini.rviz</i>.
- <b>use_sim_time</b>: use simulation time or not, default <i>true</i>.
- <b>x_pose</b>: x coordinate of the robot at the start, default <i>0.0</i>.
- <b>x_pose</b>: x coordinate of the robot at the start, default <i>0.0</i>.
- <b>yaw angle</b>: yaw angle of the robot at the start, default <i>3.14</i>.
- <b>world_name</b>: name of the used world, the world must be in the world folder, default <i>empty.world</i>.