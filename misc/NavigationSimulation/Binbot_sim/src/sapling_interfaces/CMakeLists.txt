cmake_minimum_required(VERSION 3.8)
project(sapling_interfaces)
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/LeaderPose.msg"
  "msg/FollowerStatus.msg"
  "srv/AreaCoords.srv"
)
ament_package()