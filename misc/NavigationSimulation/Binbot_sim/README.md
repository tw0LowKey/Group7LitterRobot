# Binbot Sim — Dual-Robot Litter Pickup Simulation

A ROS 2 Humble simulation of two coordinated mobile robots performing autonomous litter collection in a warehouse environment. The **leader** sweeps a GPS-defined area on a lawnmower path and collects litter; the **follower** trails behind and acts as a mobile bin, parking beside the leader whenever litter is picked up.

---

## Table of Contents

1. [System Overview](#1-system-overview)
2. [Prerequisites](#2-prerequisites)
3. [Repository Structure](#3-repository-structure)
4. [Build Instructions](#4-build-instructions)
5. [Running the Simulation](#5-running-the-simulation)
6. [Node Reference](#6-node-reference)
7. [Topic & Service Reference](#7-topic--service-reference)
8. [Custom Interfaces](#8-custom-interfaces)
9. [Key Parameters](#9-key-parameters)
10. [Litter Pickup Flow — Step by Step](#10-litter-pickup-flow--step-by-step)
11. [Testing Litter Detection Manually](#11-testing-litter-detection-manually)
12. [Troubleshooting](#12-troubleshooting)

---

## 1. System Overview

```
┌─────────────────────────────────────────────────────────────┐
│                        LEADER ROBOT                         │
│                                                             │
│  waypoint_generator  ──/waypoints──►  leader_navigation     │
│        ▲                                    │               │
│   /area_coord                        /pause_navigation      │
│   (service)                          /resume_navigation     │
│                                             │               │
│  litter_handler  ◄──/vision/detected_litter (camera/sim)    │
│        │                                                    │
│   /call_bin ──►  follower_coordination  ◄──/comms/          │
│                        │                follower_to_        │
│                  /comms/leader_to_       leader_tx          │
│                  follower_tx ──────────────────────────┐    │
└────────────────────────────────────────────────────────│────┘
                                                         │
┌────────────────────────────────────────────────────────▼────┐
│                       FOLLOWER ROBOT                        │
│                                                             │
│         gps_follower_behavior  ──►  Nav2 (/follower/...)    │
│              (breadcrumb trail / parking)                   │
└─────────────────────────────────────────────────────────────┘
```

**What happens end to end:**

1. The operator calls the `/area_coord` service with GPS corner coordinates.
2. `waypoint_generator_node` converts the area into a lawnmower grid of XY waypoints and publishes them.
3. `leader_navigation_node` drives the leader robot through those waypoints using Nav2's `FollowWaypoints` action.
4. `gps_follower_behavior_node` keeps the follower robot trailing the leader via a GPS breadcrumb trail.
5. When litter is detected on `/vision/detected_litter`, `litter_handler_node` pauses the leader, drives it to the litter, and signals the follower bin.
6. `follower_coordination_node` relays the call to the follower; the follower parks directly behind the leader.
7. After a configurable pickup delay, the follower is released and the leader resumes its sweep.

---

## 2. Prerequisites

| Requirement | Version |
|---|---|
| Ubuntu | 22.04 |
| ROS 2 | Humble |
| Gazebo (Ignition) | Fortress |
| Python | 3.10+ |

### ROS 2 packages

```bash
sudo apt install \
  ros-humble-navigation2 \
  ros-humble-nav2-bringup \
  ros-humble-pointcloud-to-laserscan \
  ros-humble-slam-toolbox \
  ros-humble-joint-state-publisher-gui \
  ros-humble-robot-state-publisher \
  ros-humble-xacro \
  ros-humble-ros-gz-bridge \
  ros-humble-ros-gz-sim \
  ros-humble-topic-tools
```

---

## 3. Repository Structure

```
Binbot_sim/
├── src/
│   ├── robot_description/          # URDF/xacro models, Gazebo world, bridge configs
│   ├── rm_navigation/              # Nav2 config, maps, launch files for leader Nav2
│   ├── rm_localization/            # Localisation / SLAM launch helpers
│   ├── sapling_interfaces/         # Custom ROS 2 messages and services
│   ├── follower_nav2_pkg/          # All behaviour nodes (leader + follower logic)
│   └── aws-robomaker-small-warehouse-world/  # (optional) warehouse Gazebo world
├── Dockerfile                      # Docker image definition
├── docker-compose.yaml             # Multi-service Docker Compose
└── README.md                       # This file
```

### Packages at a glance

| Package | Purpose |
|---|---|
| `robot_description` | URDF/xacro for both robots, empty GPS Gazebo world, Gz↔ROS 2 bridge configs |
| `rm_navigation` | Nav2 parameter files, pre-built map, RViz config, leader/follower Nav2 launch files |
| `rm_localization` | SLAM Toolbox + AMCL localisation launch files |
| `sapling_interfaces` | `LeaderPose.msg`, `FollowerStatus.msg`, `AreaCoords.srv` |
| `follower_nav2_pkg` | Five behaviour nodes + launch file |

---

## 4. Build Instructions

```bash
# 1. Source ROS 2
source /opt/ros/humble/setup.bash

# 2. Move into the workspace root


# 3. Install dependencies
rosdep update
rosdep install --from-paths src --ignore-src -r -y

# 4. Build
colcon build --cmake-clean-cache

# 5. Source the workspace overlay
source install/setup.bash
```



## 5. Running the Simulation

### Option A — Single command (recommended)

If you just want everything up and running, one launch file does it all:

```bash
source install/setup.bash
ros2 launch rm_navigation full_system_launch.py
```

This starts everything in the correct order with automatic delays between each stage:

| Delay | What starts |
|---|---|
| 0 s | Gazebo + both robots (`dual_robot.launch.py`) |
| +10 s | Leader Nav2 stack |
| +15 s | Follower Nav2 stack (localization, no SLAM) |
| +25 s | All behaviour nodes (`gps_behavior_launch.py`) |

Once it has settled (~30 s), call the `/area_coord` service to begin the sweep (see [Start the sweep](#start-the-sweep) below).

> **Note:** The full system launch runs the follower Nav2 with `slam:=false` (uses the pre-built map). If you need SLAM for the leader instead, use the manual launch below.

---

### Option B — Manual launch (four terminals)

Use this if you need more control — e.g. to swap SLAM on/off or restart one stack independently. Source the workspace (`source install/setup.bash`) in each terminal before running.

### Terminal 1 — Gazebo + both robots

```bash
source install/setup.bash
ros2 launch robot_description dual_robot.launch.py
```

This starts:
- Ignition Gazebo with the empty GPS world (`empty_gps.sdf`)
- Leader robot spawned at (0, 0)
- Follower robot spawned at (-0.8, 0)
- ROS ↔ Gazebo topic bridges for both robots

### Terminal 2 — Leader Nav2 stack

```bash
source install/setup.bash
ros2 launch rm_navigation rm_navigation.launch.py
```

Launches Nav2 (SLAM by default), RViz, and a static `world → map` TF.

Optional arguments:

| Argument | Default | Options |
|---|---|---|
| `slam` | `true` | `true` / `false` |
| `localization` | `amcl` | `amcl` / `slam_toolbox` |

To use the pre-built map instead of SLAM:

```bash
ros2 launch rm_navigation rm_navigation.launch.py slam:=false
```

### Terminal 3 — Follower Nav2 stack

```bash
source install/setup.bash
ros2 launch rm_navigation rm_navigation_follower.launch.py slam:=false
```

### Terminal 4 — Behaviour nodes

```bash
source install/setup.bash
ros2 launch follower_nav2_pkg gps_behavior_launch.py
```

Starts all behaviour nodes:
- `waypoint_generator_node`
- `leader_navigation_node`
- `litter_handler_node`
- `follower_coordination_node`
- `gps_follower_behavior_node`
- `litter_trigger_node` (automated test triggers)

### Start the sweep

Once all nodes are running, tell the system which area to sweep by calling the `/area_coord` service with the GPS coordinates of two opposite corners:

```bash
source install/setup.bash
ros2 service call /area_coord sapling_interfaces/srv/AreaCoords "{
  top_left_latitude: 53.48080,
  top_left_longitude: -2.24260,
  bottom_right_latitude: 53.48075,
  bottom_right_longitude: -2.24252
}"
```

The node will respond with the number of waypoints generated and the leader will start moving immediately.

---

## 6. Node Reference

All behaviour nodes live in `src/follower_nav2_pkg/follower_nav2_pkg/`.

### `waypoint_generator_node`

Converts a GPS bounding box into a lawnmower grid of XY waypoints.

- Listens for the first `/navsat` GPS fix to set a coordinate origin (reference point).
- Exposes the `/area_coord` service. When called, generates rows spaced `strip_width` metres apart and publishes a `PoseArray` on `/waypoints` with `TRANSIENT_LOCAL` QoS so late subscribers still receive it.
- Each waypoint's orientation is set to face along the row direction.

### `leader_navigation_node`

Drives the leader robot through the generated waypoints.

- Waits for `/waypoints`, then sends the full list to Nav2's `FollowWaypoints` action server.
- Tracks the current waypoint index from feedback and publishes the **travel heading** (direction of movement) on `/leader/travel_heading`.
- Handles **pause** and **resume** signals from the litter handler by cancelling and re-submitting the batch goal at the correct index.
- Publishes `/sweep_done` when all waypoints are completed.

**States:** `WAITING_WAYPOINTS → NAVIGATING ↔ PAUSED → IDLE`

### `litter_handler_node`

Manages the three-phase litter approach sequence whenever litter is detected.

1. **Phase 1 — Navigate:** Drives to an approach pose `approach_offset` metres from the litter, facing it.
2. **Phase 2 — Rotate:** Rotates in place to face the litter precisely (fine heading correction).
3. **Phase 3 — Align:** Rotates to face the travel heading (readies the leader to resume the sweep).
4. Publishes `/call_bin`, then waits for `/start_navigation` before continuing.

Multiple detections are queued. When the queue drains the leader resumes its waypoint sweep.

**States:** `IDLE → PAUSING → NAVIGATING → ROTATING → ALIGNING → WAITING_PICKUP → (loop or IDLE)`

### `follower_coordination_node`

Communication bridge on the **leader side**.

- Broadcasts the leader's GPS + orientation + `call_bin` flag on `/comms/leader_to_follower_tx` at 2 Hz.
- Receives `FollowerStatus` from the follower.
- When `call_bin` is active and the follower freshly reports `parked=True`, starts a `pickup_wait_time` timer.
- When the timer fires, clears the flag and publishes `/start_navigation` to the litter handler.

### `gps_follower_behavior_node`

Controls the follower robot's movement via three states.

- **FOLLOWING:** Drops GPS breadcrumbs every `breadcrumb_distance` metres as the leader moves. Follows them to maintain a trail distance ≥ `min_follow_distance`. Breadcrumbs are converted from the GPS-XY frame to the odom/map frame before being sent to Nav2.
- **PARKING:** Computes a target directly behind the leader (`leader_half_length + park_distance + follower_half_length`) and drives there. Publishes `parked=True` and `bin_ready=True` on arrival.
- **WAITING:** Holds position. Resumes FOLLOWING when the leader moves away by more than `resume_distance`.

### `litter_trigger_node`

Automated test helper. Publishes fake litter detections on `/vision/detected_litter` when the leader reaches predefined positions during a sweep. Each trigger fires exactly once. Useful for repeatable testing without a real vision pipeline.

---

## 7. Topic & Service Reference

### Topics

| Topic | Type | Direction | Description |
|---|---|---|---|
| `/waypoints` | `geometry_msgs/PoseArray` | generator → nav | Lawnmower waypoints (TRANSIENT_LOCAL) |
| `/odom` | `nav_msgs/Odometry` | Gazebo → nodes | Leader odometry |
| `/follower/odom` | `nav_msgs/Odometry` | Gazebo → nodes | Follower odometry |
| `/navsat` | `sensor_msgs/NavSatFix` | Gazebo → nodes | Leader GPS |
| `/follower/navsat` | `sensor_msgs/NavSatFix` | Gazebo → nodes | Follower GPS |
| `/vision/detected_litter` | `geometry_msgs/PoseStamped` | camera/sim → handler | Litter position in map frame |
| `/pause_navigation` | `std_msgs/Bool` | handler → leader nav | Pause waypoint sweep |
| `/resume_navigation` | `std_msgs/Bool` | handler → leader nav | Resume waypoint sweep |
| `/leader/travel_heading` | `std_msgs/Float64` | leader nav → handler | Current heading (radians) |
| `/call_bin` | `std_msgs/Bool` | handler → coordination | Leader at litter, request follower |
| `/start_navigation` | `std_msgs/Bool` | coordination → handler | Pickup done, continue |
| `/sweep_done` | `std_msgs/Bool` | leader nav → coordination | All waypoints complete |
| `/comms/leader_to_follower_tx` | `sapling_interfaces/LeaderPose` | coordination → follower | Leader pose + call_bin flag |
| `/comms/follower_to_leader_tx` | `sapling_interfaces/FollowerStatus` | follower → coordination | Follower parked/bin status |
| `/follower_status` | `sapling_interfaces/FollowerStatus` | coordination → local | Echo of follower status |
| `/bin_ready` | `std_msgs/Bool` | follower → — | Follower bin in position |

### Services

| Service | Type | Description |
|---|---|---|
| `/area_coord` | `sapling_interfaces/AreaCoords` | Provide GPS corners → generate waypoints |

---

## 8. Custom Interfaces

Defined in `src/sapling_interfaces/`.

### `sapling_interfaces/msg/LeaderPose`

Published by the coordination node to the follower at 2 Hz.

```
int32   seq             # Monotonically increasing sequence number
float64 latitude        # Leader GPS latitude
float64 longitude       # Leader GPS longitude
float32 orientation_z   # Quaternion z (yaw)
float32 orientation_w   # Quaternion w (yaw)
bool    call_bin        # True: leader needs the bin robot now
```

### `sapling_interfaces/msg/FollowerStatus`

Published by the follower back to the leader at 2 Hz.

```
int32   seq       # Monotonically increasing sequence number
bool    parked    # True when follower has reached its park position
float64 park_x   # Follower GPS-XY X at time of parking
float64 park_y   # Follower GPS-XY Y at time of parking
bool    bin_ready # True when bin is in position
```

### `sapling_interfaces/srv/AreaCoords`

```
# Request
float64 top_left_latitude
float64 top_left_longitude
float64 bottom_right_latitude
float64 bottom_right_longitude
---
# Response
bool    success
int32   num_waypoints
```

Corner order does not matter — the node auto-corrects the bounding box.

---

## 9. Key Parameters

### `waypoint_generator_node`

| Parameter | Default | Description |
|---|---|---|
| `strip_width` | `1.35` | Distance between lawnmower rows (metres) |
| `auto_ref` | `true` | Use first GPS fix as coordinate origin |
| `gps_topic` | `/navsat` | Leader GPS topic |

### `litter_handler_node`

| Parameter | Default | Description |
|---|---|---|
| `approach_offset` | `0.3` | Stop this far from litter (metres) |
| `pause_settle_time` | `1.0` | Seconds to wait after pause before taking Nav2 |

### `follower_coordination_node`

| Parameter | Default | Description |
|---|---|---|
| `pickup_wait_time` | `2.0` | Simulated pickup duration (seconds) |

### `gps_follower_behavior_node`

| Parameter | Default | Description |
|---|---|---|
| `breadcrumb_distance` | `0.5` | Drop a breadcrumb every N metres |
| `min_follow_distance` | `1.5` | Do not move if trail distance is below this |
| `park_distance` | `0.1` | Gap between robots when parked (metres) |
| `resume_distance` | `1.5` | Resume following when leader is this far away |
| `leader_half_length` | `0.325` | Half-length of leader robot body (metres) |
| `follower_half_length` | `0.325` | Half-length of follower robot body (metres) |

---

## 10. Litter Pickup Flow — Step by Step

```
[Vision]  publishes PoseStamped on /vision/detected_litter
    │
    ▼
[litter_handler]  queues the pose
    │  (if IDLE) publishes /pause_navigation
    │                        │
    │                        ▼
    │              [leader_navigation]  cancels FollowWaypoints batch
    │
    │  waits pause_settle_time (1 s) for Nav2 to fully stop
    │
    ▼  STATE: NAVIGATING
[litter_handler]  sends NavigateToPose → approach pose (approach_offset m from litter)
    │
    ▼  Phase 1 complete  →  STATE: ROTATING
[litter_handler]  sends NavigateToPose → same XY, rotate to face litter
    │
    ▼  Phase 2 complete  →  STATE: ALIGNING
[litter_handler]  sends NavigateToPose → same XY, rotate to travel heading
    │
    ▼  Phase 3 complete  →  STATE: WAITING_PICKUP
[litter_handler]  publishes /call_bin
    │
    ▼
[follower_coordination]  sets call_bin_flag, broadcasts via LeaderPose
    │
    ▼
[gps_follower_behavior]  receives call_bin → STATE: PARKING
    │  cancels breadcrumb goal
    │  drives to (leader_pos − heading × total_offset)
    │
    ▼  parked  →  STATE: WAITING
[gps_follower_behavior]  publishes FollowerStatus(parked=True, bin_ready=True)
    │
    ▼
[follower_coordination]  sees fresh parked=True → starts pickup_wait_time timer
    │
    ▼  timer fires
[follower_coordination]  clears call_bin_flag, publishes /start_navigation
    │
    ▼
[litter_handler]  receives /start_navigation
    │  if queue empty  →  publishes /resume_navigation
    │                        │
    │                        ▼
    │              [leader_navigation]  re-submits remaining waypoints
    │
    │  if more litter  →  processes next item from queue
    │
    ▼
[gps_follower_behavior]  leader moves away (> resume_distance) → STATE: FOLLOWING
```

---

## 11. Testing Litter Detection Manually

The `litter_trigger_node` fires automatically at predefined leader positions during a sweep. To inject litter at any time:

```bash
# First litter detection
ros2 topic pub --once /vision/detected_litter geometry_msgs/msg/PoseStamped "{
  header: {frame_id: 'map'},
  pose: {position: {x: 4.0, y: -2.0, z: 0.0}, orientation: {w: 1.0}}
}"

# Second detection — publish within a few seconds to test queue behaviour
ros2 topic pub --once /vision/detected_litter geometry_msgs/msg/PoseStamped "{
  header: {frame_id: 'map'},
  pose: {position: {x: 4.0, y: 0.5, z: 0.0}, orientation: {w: 1.0}}
}"
```

### Monitoring commands

```bash
# Watch litter/queue/pickup log messages
ros2 topic echo /rosout --field msg | grep -i "litter\|queue\|pickup\|resume"

# Check leader position
ros2 topic echo /odom --once --field pose.pose.position

# Check follower position
ros2 topic echo /follower/odom --once --field pose.pose.position

# See follower state
ros2 topic echo /follower_status
```

---

## 12. Troubleshooting

**Robots do not move after calling `/area_coord`**
- Both Nav2 stacks must be running before calling the service.
- Confirm the GPS topic is active: `ros2 topic hz /navsat`. The reference point is set from the first valid fix — if it never arrives, waypoints cannot be projected to XY.

**Follower does not trail the leader**
- Breadcrumbs are only dropped once the reference point is set from the first `LeaderPose` message. Check `/comms/leader_to_follower_tx` is publishing and look for `Reference point set from leader` in the follower node's logs.

**Leader gets stuck at a waypoint**
- Nav2 may have aborted a goal due to an obstacle or planner failure. The node skips the failed waypoint and continues automatically. Check `/rosout` for `FollowWaypoints aborted`.

**`area too narrow after applying 0.5m turn buffer`**
- The east–west span of the area is too small. Ensure the longitude range covers at least ~1 metre.

**Kill all simulation processes cleanly**

```bash
killall -9 ros2 rviz2 component_container_isolated nav2_container
rm -rf ~/.gazebo/log/*
```
