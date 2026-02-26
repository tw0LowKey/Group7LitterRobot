Scout Navigation Simulation (ROS2 Humble)

Multi-package ROS2 workspace for simulating and navigating a Scout robot using Gazebo, Nav2, RViz, and a custom waypoint navigation node.

Requirements

Ubuntu 22.04
ROS2 Humble
Nav2
Gazebo 
Colcon

Workspace Setup

In the root of the workspace:

source /opt/ros/humble/setup.bash
colcon build --cmake-clean-cache
source install/setup.bash
Running the System

Each of the following commands must be run in separate terminals.

Terminal 1 – Launch Gazebo Simulation
This launches the Scout robot inside the simulation environment.

ros2 launch robot_description simulation.launch.py

Terminal 2 – Launch Nav2 and RViz
This starts the Nav2 stack and RViz for planning, localization, and visualization.

ros2 launch rm_navigation rm_navigation.launch.py

Terminal 3 – Start Waypoint Navigation Node
Runs the custom waypoint navigation package.

ros2 run waypoint_navigation_pkg waypoint_navigation_node

Terminal 4 – Trigger Navigation
Triggers the waypoint execution using a service call.

ros2 service call /start_navigation std_srvs/srv/Trigger
Execution Flow

Gazebo launches the robot in simulation.

Nav2 initializes planners, controllers, and localization in RViz.

The waypoint navigation node waits for a trigger.

The service call starts waypoint-based autonomous navigation.

Workspace Structure
scout_ws1/
 ├── src/
 │   ├── robot_description/
 │   ├── rm_navigation/
 │   ├── rm_localization/
 │   ├── waypoint_navigation_pkg/
 │   └── aws-robomaker-small-warehouse-world/
 ├── Dockerfile
 ├── docker-compose.yaml
 └── README.md