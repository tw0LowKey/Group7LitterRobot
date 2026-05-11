cmake_minimum_required(VERSION 3.8)
project(sapling_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find the ROS 2 CMake build tools
find_package(ament_cmake REQUIRED)

# Install the directories so ROS 2 can find them at runtime
install(DIRECTORY
  urdf
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
