cmake_minimum_required(VERSION 3.5)
project(rm_localization)

find_package(ament_cmake REQUIRED)
find_package(nav2_common REQUIRED)
find_package(navigation2 REQUIRED)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
	if("cxx_std_17" IN_LIST CMAKE_CXX_COMPILE_FEATURES)
		set(CMAKE_CXX_STANDARD 17)
	else()
		message(FATAL_ERROR "cxx_std_17 could not be found.")
	endif()
endif()

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU" OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
	add_compile_options(-Wall -Wextra -Wpedantic -Werror -Wdeprecated -fPIC -Wshadow -Wnull-dereference)
	add_compile_options("$<$<COMPILE_LANGUAGE:CXX>:-Wnon-virtual-dtor>")
endif()

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/fake_gps_from_odom.py
  scripts/fake_fixed_gps_node.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()