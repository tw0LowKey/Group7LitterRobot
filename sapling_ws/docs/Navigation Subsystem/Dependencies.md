## LiDAR Setup and Integration

The navigation subsystem uses a YDLIDAR X4-Pro 2D LiDAR to provide obstacle information to the ROS2 navigation stack. The LiDAR publishes scan data on the `/scan` topic as `sensor_msgs/LaserScan`, which is then used by RViz2 for visualisation and by Nav2 costmaps for obstacle detection and avoidance.

### YDLIDAR Driver Installation

The YDLIDAR ROS2 driver requires the YDLIDAR SDK to be installed first. The SDK was installed and built from source using:

```bash
cd ~
git clone https://github.com/YDLIDAR/YDLidar-SDK.git
mkdir -p YDLidar-SDK/build
cd YDLidar-SDK/build
cmake ..
make -j$(nproc)
sudo make install
sudo ldconfig
```

A separate ROS2 workspace was then created for the LiDAR driver:

```bash
mkdir -p ~/ydlidar_ros2_ws/src
cd ~/ydlidar_ros2_ws/src
git clone https://github.com/YDLIDAR/ydlidar_ros2_driver.git -b humble
```

The workspace was built using:

```bash
cd ~/ydlidar_ros2_ws
source /opt/ros/humble/setup.bash
rosdep update
rosdep install -y -i --from-paths src --rosdistro humble
colcon build --symlink-install
source install/setup.bash
```

### Serial Permission and Port Check

The LiDAR connects through a serial USB port. To allow access to the port, the user must be added to the `dialout` group:

```bash
sudo usermod -a -G dialout $USER
```

After this, the system should be logged out and logged back in, or rebooted. The connected LiDAR port can be checked using:

```bash
ls -l /dev/ttyUSB*
```

The LiDAR is usually detected as:

```bash
/dev/ttyUSB0
```

### Launching the LiDAR Driver

The LiDAR driver is launched using the X4-Pro parameter file:

```bash
source /opt/ros/humble/setup.bash
source ~/ydlidar_ros2_ws/install/setup.bash

ros2 launch ydlidar_ros2_driver ydlidar_launch.py \
params_file:=$HOME/ydlidar_ros2_ws/src/ydlidar_ros2_driver/params/X4-Pro.yaml
```

A successful launch should show that the LiDAR is connected and scanning.

### Checking the `/scan` Topic

In a new terminal, the `/scan` topic should be checked:

```bash
source /opt/ros/humble/setup.bash
source ~/ydlidar_ros2_ws/install/setup.bash

ros2 topic list | grep scan
```

The expected topic is:

```bash
/scan
```

The scan rate can be checked using:

```bash
ros2 topic hz /scan
```

The expected scan rate is approximately 10 Hz.

### RViz2 Visualisation

RViz2 can be used to confirm that the LiDAR scan is being published correctly:

```bash
rviz2
```

In RViz2:

1. Set the fixed frame to:

```text
base_link
```

If this does not work, use:

```text
laser_frame
```

2. Add a LaserScan display using:

```text
Add → By Topic → /scan → LaserScan
```

3. In the LaserScan display settings, change the Reliability Policy to:

```text
Best Effort
```

This is important because the LiDAR publishes using Best Effort QoS. If RViz2 is not also set to Best Effort, the scan may not appear due to an incompatible QoS policy.

When working correctly, RViz2 should show a circular scan pattern, and obstacles such as a hand placed in front of the LiDAR should appear immediately.

### TF Frame Check

For the LiDAR data to be used correctly, the TF transform between the robot base and the LiDAR frame must exist. This can be checked using:

```bash
ros2 run tf2_ros tf2_echo base_link laser_frame
```

If the transform is printed successfully, the LiDAR frame is correctly connected to the robot base frame.

The required TF chain for Nav2 is:

```text
map → odom → base_link → laser_frame
```

### Nav2 Costmap Integration

The `/scan` topic is used as an obstacle input for the Nav2 local and global costmaps. The Nav2 parameter file should be opened, for example:

```bash
scout_ws1/src/.../nav2_params.yaml
```

Inside both the `local_costmap` and `global_costmap` sections, the obstacle layer should be configured to use the `/scan` topic:

```yaml
obstacle_layer:
  plugin: "nav2_costmap_2d::ObstacleLayer"
  enabled: True
  observation_sources: scan

  scan:
    topic: /scan
    max_obstacle_height: 2.0
    clearing: True
    marking: True
    data_type: "LaserScan"
    raytrace_max_range: 10.0
    obstacle_max_range: 10.0
    qos_overrides:
      reliability: best_effort
```

The `qos_overrides` setting is important because the LiDAR publishes scan data using Best Effort reliability. Without this setting, Nav2 may not receive the LiDAR data correctly.

The costmap frames should also be checked:

```yaml
global_frame: map
robot_base_frame: base_link
```

The LaserScan frame can be checked using:

```bash
ros2 topic echo /scan --once | grep frame_id
```

The expected frame is:

```text
laser_frame
```

This is correct as long as the TF chain between `base_link` and `laser_frame` exists.

### Launch Order

The LiDAR driver should be launched before starting Nav2.

Terminal 1:

```bash
source /opt/ros/humble/setup.bash
source ~/ydlidar_ros2_ws/install/setup.bash

ros2 launch ydlidar_ros2_driver ydlidar_launch.py \
params_file:=$HOME/ydlidar_ros2_ws/src/ydlidar_ros2_driver/params/X4-Pro.yaml
```

Terminal 2:

```bash
source /opt/ros/humble/setup.bash
source ~/ydlidar_ros2_ws/install/setup.bash
source ~/scout_ws1/install/setup.bash

ros2 launch <your_nav2_launch>
```

### Testing Obstacle Detection in Nav2

To confirm that Nav2 is receiving LiDAR obstacle data, open the Nav2 RViz configuration and display:

* LaserScan
* Local Costmap
* Global Costmap

When an object is placed in front of the LiDAR, red obstacle cells should appear in the costmap. This confirms that the LiDAR scan is being used by Nav2 for obstacle detection.

If Nav2 does not respond to obstacles, the `/scan` topic information should be checked:

```bash
ros2 topic info /scan -v
```

The Nav2 costmap subscriber should appear, and its reliability policy should be:

```text
best_effort
```

If the subscriber is not visible or the QoS does not match, Nav2 may not be receiving the LiDAR data.


## Scout Mini Driver Setup

The Scout Mini base is controlled using the ROS2 Scout driver. This driver provides the `scout_base` package, which is used to communicate with the Scout Mini through the CAN interface.

### Cloning the Scout ROS2 Driver

The Scout driver can be cloned into the main ROS2 workspace source folder:

```bash
cd ~/Group7LitterRobot/sapling_ws/src
git clone https://github.com/agilexrobotics/scout_ros2.git
```

After cloning, rebuild the workspace:

```bash
cd ~/Group7LitterRobot/sapling_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

The package can be checked using:

```bash
ros2 pkg list | grep scout
```

If the driver has built correctly, packages such as `scout_base`, `scout_description`, and `scout_msgs` should be visible.

### CAN Interface Setup

Before the Scout Mini can be launched, the CAN interface must be configured. In the final setup, the CAN interface is named `can_scout`:

```bash
sudo ip link set can_scout down
sudo ip link set can_scout type can bitrate 500000
sudo ip link set can_scout up
```

The Scout Mini base driver can then be launched by itself using:

```bash
source /opt/ros/humble/setup.bash
source ~/Group7LitterRobot/sapling_ws/install/setup.bash

ros2 launch scout_base scout_mini_base.launch.py port_name:=can_scout
```

This standalone launch is useful for testing whether the Scout Mini base is communicating correctly and publishing odometry on `/odom`.

### Use in the Final Integrated Launch

In the final integrated system, the Scout Mini base driver does not need to be launched manually. It is already included inside the main integrated launch file:

```bash
ros2 launch rm_navigation full_indoor_navigation.launch.py
```

Inside this launch file, the Scout Mini base driver is launched using the `scout_base` package and the `can_scout` CAN interface. Therefore, during the final demo, the user only needs to configure the CAN interface first and then run the main integrated launch.

The final flow is:

```bash
sudo ip link set can_scout down
sudo ip link set can_scout type can bitrate 500000
sudo ip link set can_scout up

source /opt/ros/humble/setup.bash
source ~/Group7LitterRobot/sapling_ws/install/setup.bash

ros2 launch rm_navigation full_indoor_navigation.launch.py
```

This starts the Scout Mini base driver as part of the complete navigation system, along with the LiDAR, robot description, Nav2 SLAM navigation, litter navigation nodes, and RViz2.


### Additional Navigation Dependencies and Checks

The integrated navigation launch depends on several ROS2 packages being available in the workspace or installed through apt. Nav2 requires the main Navigation2 stack, SLAM Toolbox is used for indoor SLAM, and `robot_localization` can be used when filtered odometry or sensor fusion is required. The Scout Mini base also requires CAN communication tools to configure and test the CAN interface.

The required system dependencies can be installed using:

```bash
sudo apt update
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup
sudo apt install ros-humble-slam-toolbox
sudo apt install ros-humble-robot-localization
sudo apt install ros-humble-tf-transformations
sudo apt install can-utils
sudo apt install python3-colcon-common-extensions python3-rosdep
