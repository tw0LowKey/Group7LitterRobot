# Comms Package

The comms package is responsible for handling communication between the Sapling robots (pickbot/binbot) and external systems, as well as inter-robot communication. It includes nodes for bridging LoRa messages, executing commands, and streaming video.

## Overview

This package provides the following key functionalities:
- **BLE Provisioning & LoRa Bridge**: A BLE server that handles initial robot provisioning (receiving keys, protocol definitions, and node IDs) before switching over to a physical LoRa radio hardware interface.
- **Command Execution**: Processing and executing commands received via LoRa.
- **Status Reporting**: Sending telemetry data (battery, GPS).
- **Video Streaming**: Bridging ROS image topics to an RTSP stream for remote monitoring.
## Nodes

### 1. sapling_comms_node
A ROS 2 node that handles dual-stage communication: initial BLE configuration/provisioning followed by active runtime LoRa communication using RFM9X hardware via SPI.

- **Startup / BLE Provisioning**:
  - Starts a BLE Server using the `Bless` library, advertising with the name defined by the `SAPLING_ID` environment variable.
  - Listens on Service UUID: `036f33e0-9573-4b0e-88d1-18af960d5a95` and Characteristic UUID: `92eda5fb-c187-4f41-aaf2-3931b9cb4c56`.
  - Expects a JSON payload from a client containing `loraNodeId`, `secretKey`, and `protocol`. Once successfully received, it shuts down BLE and spins up the LoRa thread.

- **Runtime / LoRa Interface**:
  - Uses an Adafruit RFM9X module over SPI (`board.SCK`, `board.MOSI`, `board.MISO`, CS pin `board.CE1`, Reset pin `board.D13`) at 433.0 MHz.
  - Automatically handles message acknowledgements (`send_with_ack` and `receive(with_ack=True)`).

- **Topics Published**:
  - `/comms/lora_rx` (`std_msgs/msg/String`): Incoming packets decoded into JSON using `sapling_shared.py`.
  - `/comms/comms_protocol` (`std_msgs/msg/String`): The active JSON protocol configuration received during BLE provisioning.
- **Topics Subscribed**:
  - `/comms/lora_tx` (`sapling_interfaces/msg/LoraTransmission`): Outgoing messages pushed into a safe background thread queue for LoRa transmission.

### 2. sapling_executor_node
The central execution node that handles commands and manages robot status. It requires the `SAPLING_ROLE` environment variable to be set (either `pickbot` or `binbot`).

- **Key Responsibilities**:
  - Heartbeat generation (lat, lng, battery).
  - Movement command smoothing and ramping for `/cmd_vel`.
  - Managing area coordinates and litter markers.
  - Leader-Follower communication bridging.
- **Environment Variables**:
  - `SAPLING_ROLE`: Must be set to `pickbot` or `binbot`.

### 3. sapling_video_node
Bridges ROS image topics to an FFmpeg-based RTSP stream. This is typically used in conjunction with [MediaMTX](https://github.com/bluenviron/mediamtx).

- **Parameters**:
  - `image_topic`: The ROS topic to read images from (default: `/camera/color/image_raw`).
  - `rtsp_url`: The destination RTSP URL (default: `rtsp://127.0.0.1:8554/robot`).
  - `fps`: Target frames per second.
  - `bitrate`: Stream bitrate (e.g., `3000k`).

### 4. sapling_area_coords_test_node
A utility node used for testing area coordinate services.

## Directory Structure Notes

- **comms/actual/**: Contains alternative or archived versions of the core nodes. These may be used for specific testing scenarios or reference.

## Shared Logic

- **sapling_shared.py**: Contains utility functions for encoding and decoding binary packets based on the defined communication protocol.

## Launch Files

- **teleop_test_basic.launch.py**: Launches `sapling_comms_node`, `sapling_executor_node`, and `web_video_server`.
- **teleop_test_video.launch.py**: Launches the full suite including `sapling_video_node` (configured for MediaMTX).

## Setup & Usage

### Prerequisites
- Install the required python packages:
  ```bash
  pip install -r requirements.txt
  ```
- Install the required ROS2 packages:
  ```bash
  sudo apt install ros-$ROS_DISTRO-cv-bridge -y
  ```
- `sapling_interfaces` package must be built and sourced.
- For video streaming, MediaMTX should be running (e.g., via Docker):
  ```bash
  sudo docker run --rm -it --network=host bluenviron/mediamtx:latest
  ```
- Connect the RFM9x LoRa module and the emergency stop button to the Jetson Orin Nano:
  ![Jetson Orin Nano Wiring Diagram](wiring_diagram.svg)

### Running the Nodes
To start the basic teleop stack:
```bash
export SAPLING_ID=R-067  # Has to comply with with R-### where ### is a 3 digit number
export SAPLING_ROLE=pickbot  # or binbot
ros2 launch comms teleop_test_basic.launch.py
```

To start the stack with video streaming:
```bash
export SAPLING_ID=R-067  # Has to comply with with R-### where ### is a 3 digit number
export SAPLING_ROLE=pickbot
ros2 launch comms teleop_test_video.launch.py
```
