## Communications Subsystem Startup

For startup of the Communications Subsystem, ensure the RFM9x LoRa module is correctly wired to the Jetson Orin Nano.

### Environment Variables
Before running the nodes, the following environment variables must be set:
- `SAPLING_ID`: The unique identifier for the robot (e.g., `R-067`). It must follow the format `R-###`.
- `SAPLING_ROLE`: The role of the robot, either `pickbot` or `binbot`.

Example:
```bash
export SAPLING_ID=R-067
export SAPLING_ROLE=pickbot
```

### Starting the Subsystem
To start the basic teleop stack (BLE/LoRa bridge and command executor):
```bash
cd ~/Group7LitterRobot/sapling_ws
source install/setup.bash
ros2 launch comms teleop_test_basic.launch.py
```

To start the stack with video streaming (requires MediaMTX to be running):
```bash
cd ~/Group7LitterRobot/sapling_ws
source install/setup.bash
ros2 launch comms teleop_test_video.launch.py
```

### BLE Provisioning
Upon startup, the `sapling_comms_node` will start a BLE Server.
1. It advertises with the name set in `SAPLING_ID`.
2. A client must connect and send a JSON payload containing `loraNodeId`, `secretKey`, and `protocol` to the characteristic UUID `92eda5fb-c187-4f41-aaf2-3931b9cb4c56`.
3. Once provisioning is successful, the node will shut down BLE and switch to active LoRa communication.
