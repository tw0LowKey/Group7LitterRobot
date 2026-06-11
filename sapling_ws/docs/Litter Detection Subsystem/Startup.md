## Litter Detection Subsystem Startup

For startup of the Litter Detection Subsystem, ensure the Orbbec Femto Mega is connected via USB-C to the Jetson Orin Nano, and is connected to the Bluetti battery pack via DC barrel jack. The LED power indicator on the Orbbec Femto Mega should become lit white if it is correctly powered from the DC barrel jack. Power from the DC barrel jack is not necessarily required, and the Orbbec Femto Mega may be powered directly via USB-C, but this increases over-current on the Jetson Orin Nano. 

Connection to the Jetson can be confirmed via running `lsusb` in the terminal, where if the device is recognised an instance for the Orbbec Femto Mega should be listed.

Once connection is confirmed, the Litter Detection Subsystem can simply be started via running:

```bash
cd ~/Group7LitterRobot/sapling_ws
source install/setup.bash
ros2 launch computer_vision vision_start.launch.py
```

Ctrl+C may then be pressed to shutdown the subsystem. 

It should be ensured no unnecessary programs are open when attempting to run the Litter Detection Subsystem, as ample RAM is required to run the 2GB rgb_depth_node. Should the node crash, indicated via `[ERROR] [rgb_depth_node-2]: process has died` and a memory allocation error, shut down the subsystem and run it again with more memory available. 

### Ground Plane Node

Should the ground plane for grasping need to be recalculated, delete the old grasp plane node to allow regeneration, and do
```bash
ros2 run computer_vision ground_plane_node
```
which should save a new .npy ground plane file. Ensure the PickBot is positioned on flat ground, with the camera angled at solely flat ground, otherwise the result will be incorrect. See the final report reissue for further detail on the ground plane node. 
