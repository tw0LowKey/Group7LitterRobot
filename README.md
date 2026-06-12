# Group 7 — SAPLInG Litter Robot

An autonomous outdoor litter-collection robot built on ROS 2 (Humble). The robot navigates a defined area, detects litter with computer vision and a depth camera, picks it up with a robotic arm, and coordinates with a base station over LoRa/Bluetooth.

---

## Repository Layout

```
Group7LitterRobot/
├── sapling_ws/                  # Main ROS 2 workspace
│   ├── src/
│   │   ├── domains/             # High-level capability packages
│   │   │   ├── arm/             # Robotic arm (Piper) — driver, MoveIt, grasp detection
│   │   │   ├── comms/           # LoRa + Bluetooth comms with the base station
│   │   │   ├── navigation/      # Localisation, Nav2, waypoint following, litter handling
│   │   │   └── vision/          # Orbbec depth camera driver + computer-vision pipeline
│   │   ├── drivers/             # Low-level hardware drivers (GPS, etc.) — populated by install.sh
│   │   ├── misc/                # Utility nodes (emergency stop)
│   │   ├── sapling/             # Top-level launch package
│   │   ├── sapling_description/ # Robot URDF / xacro model
│   │   └── sapling_interfaces/  # Custom ROS 2 messages & services
│   ├── docs/                    # Assembly instructions, BOM, SolidWorks parts
│   ├── scripts/
│   │   ├── install.sh           # Clones external driver repos via vcstool
│   │   └── setup.sh             # Workspace build / environment setup
│   └── sapling.repos            # vcstool manifest for driver dependencies
└── misc/                        # Experiments & prototypes (not part of the main build)
    ├── arm/                     # Early arm scripting experiments
    ├── nav/scout/               # Scout robot Gazebo simulation
    ├── NavigationSimulation/    # Binbot navigation simulation
    └── Datasets.ipynb           # Dataset exploration notebook
```

---

## Domain Packages

### `domains/arm`
Controls the **Piper robotic arm** over CAN bus.

| Package | Purpose |
|---|---|
| `piper` | ROS 2 driver node for the Piper arm |
| `piper_description` | URDF / meshes |
| `piper_moveit` | MoveIt 2 config, pick-and-place pipeline, collision zones |
| `piper_sim` | Gazebo & MuJoCo simulation of the arm |
| `grasp_detection` | Computes grasp poses from point-cloud data |
| `scripts/` | CAN bus activation helpers (`can_activate.sh`, etc.) |

### `domains/comms`
Handles **wireless communication** between the robot and a remote base station.

| Node | Purpose |
|---|---|
| `sapling_comms_node` | LoRa radio TX/RX + Bluetooth GATT server for provisioning |
| `sapling_executor_node` | Receives commands from base, dispatches to other subsystems |
| `sapling_area_coords_test_node` | Dev/test node that serves fake GPS area coordinates |

### `domains/navigation`
Everything related to **where the robot goes**.

| Package | Purpose |
|---|---|
| `rm_localization` | AMCL / localisation config |
| `rm_navigation` | Nav2 config, maps, RViz configs |
| `robot_description` | Mobile base URDF |
| `aws-robomaker-small-warehouse-world` | Gazebo world used for simulation |
| `waypoint_navigation_pkg` | High-level behaviour nodes: lawnmower sweep, GPS follower, litter handler, waypoint generator, leader/follower coordination |

### `domains/vision`
**Perceives the environment** using a depth camera.

| Package | Purpose |
|---|---|
| `OrbbecSDK_ROS2` | Official Orbbec camera ROS 2 driver |
| `computer_vision` | Point-cloud filtering, ground-plane removal, litter detection and 3-D localisation |

### `sapling_interfaces`
Custom ROS 2 interface definitions shared across all packages.

| Interface | Purpose |
|---|---|
| `msg/LoraTransmission` | Wraps a LoRa radio packet |
| `msg/LeaderPose` | Leader robot pose for follower coordination |
| `msg/FollowerStatus` | Follower status report |
| `srv/AreaCoords` | Request/response for GPS bounding-box of the work area |

---

## Getting Started

```bash
# 1. Clone third-party drivers
cd sapling_ws
bash scripts/install.sh

# 2. Build and source
bash scripts/setup.sh

# 3. Launch (see sapling/launch/ for specific launch files)
ros2 launch sapling <launch_file>.py
```

Hardware requirements: Orbbec depth camera, Piper arm with CAN interface, LoRa radio module, Emlid Reach M2 GPS.

Created as a part of the University of Manchester EEE MEng Project for Group 7, 2025/2026.
