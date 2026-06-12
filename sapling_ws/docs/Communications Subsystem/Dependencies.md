## Communications Subsystem Dependencies

The following assumes a JetPack version of 6.2.1 and Python version 3.10.12.

### Python Packages
Install the required python packages from the comms package:
```bash
pip install adafruit-circuitpython-rfm9x bleak bless opencv-python
```

### ROS 2 Packages
The subsystem requires the following ROS 2 packages:
```bash
sudo apt install ros-$ROS_DISTRO-cv-bridge -y
```

Additionally, the `sapling_interfaces` package must be built and sourced within the workspace.

### External Tools
For video streaming functionality, [MediaMTX](https://github.com/bluenviron/mediamtx) is required. It is recommended to run it via Docker:
```bash
sudo docker run --rm -it --network=host bluenviron/mediamtx:latest
```

### Hardware Setup
The following hardware must be connected to the Jetson Orin Nano:
- **RFM9x LoRa Module**: Connected via SPI (`board.SCK`, `board.MOSI`, `board.MISO`, CS pin `board.CE1`, Reset pin `board.D13`).
- **Emergency Stop Button**.

Refer to the `wiring_diagram.svg` in the `src/domains/comms/` directory for detailed pin connections.
