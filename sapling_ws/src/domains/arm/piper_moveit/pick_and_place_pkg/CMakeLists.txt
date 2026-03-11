cmake_minimum_required(VERSION 3.8)
project(pick_and_place_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(piper_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)

# --- ARM NODE ---
add_executable(move_to_pose_server
  src/pick_and_place_node_main.cpp
  src/pick_and_place_node.cpp
)
target_include_directories(move_to_pose_server PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>)
ament_target_dependencies(move_to_pose_server rclcpp geometry_msgs piper_msgs moveit_ros_planning_interface)

# --- GRIPPER NODE ---
add_executable(set_grip_width_server
  src/gripper_node_main.cpp
  src/gripper_node.cpp
)
target_include_directories(set_grip_width_server PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>)
ament_target_dependencies(set_grip_width_server rclcpp geometry_msgs piper_msgs moveit_ros_planning_interface)

# --- HOME NODE ---
add_executable(move_to_home_server
  src/home_node_main.cpp
  src/home_node.cpp
)
target_include_directories(move_to_home_server PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>)
ament_target_dependencies(move_to_home_server rclcpp geometry_msgs piper_msgs moveit_ros_planning_interface)

# --- INSTALL ---
install(TARGETS
  move_to_pose_server
  set_grip_width_server
  move_to_home_server
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
