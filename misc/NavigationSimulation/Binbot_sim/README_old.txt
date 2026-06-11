source /opt/ros/humble/setup.bash
colcon build --cmake-clean-cache
source install/setup.bash
ros2 launch robot_description dual_robot.launch.py



# Terminal 2: leader Nav2 stack

source install/setup.bash
ros2 launch rm_navigation rm_navigation.launch.py

# Terminal 3: follower nav2 stack

source install/setup.bash
ros2 launch rm_navigation rm_navigation_follower.launch.py slam:=false


# Terminal 4: start goal
source install/setup.bash
ros2 launch follower_nav2_pkg gps_behavior_launch.py

# terminal 5: test service call to set area coordinates
source install/setup.bash
ros2 service call /area_coord sapling_interfaces/srv/AreaCoords "{
  top_left_latitude: 53.48080,
  top_left_longitude: -2.24260,
  bottom_right_latitude: 53.48075,
  bottom_right_longitude: -2.24252
}"

# litterflow:
source install/setup.bash
# First litter
ros2 topic pub --once /vision/detected_litter geometry_msgs/msg/PoseStamped "{header: {frame_id: 'map'}, pose: {position: {x: 4.0, y: -2.0, z: 0.0}, orientation: {w: 1.0}}}"

# Second litter — publish immediately after (within a few seconds)
ros2 topic pub --once /vision/detected_litter geometry_msgs/msg/PoseStamped "{header: {frame_id: 'map'}, pose: {position: {x: 4.0, y: 0.5, z: 0.0}, orientation: {w: 1.0}}}"




ros2 topic echo /rosout --field msg | grep -i "litter\|queue\|pickup\|resume"


# Leader odom position
ros2 topic echo /odom --once --field pose.pose.position

# Follower odom position
ros2 topic echo /follower/odom --once --field pose.pose.position


source /opt/ros/humble/setup.bash
colcon build --cmake-clean-cache
source install/setup.bash
ros2 launch rm_navigation full_system_launch.py 2>&1 | grep -v "missed its desired rate" | grep -v "Passing new path to controller"

killall -9 ros2 rviz2 component_container_isolated nav2_container
rm -rf ~/.gazebo/log/* 