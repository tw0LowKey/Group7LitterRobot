## Litter Retrieval Subsystem Startup

The litter retrieval subsystem may be activated by first ensuring the arm is connected to the Bluetti battery pack via Type G plug, also ensuring CAN is properly connected via the Hardware Setup. Second, the AC button on the Bluetti battery pack must be pressed to activate AC power, at which point green LEDs should light on the AgileX Piper Arm. 

Then to activate CAN ports run in a terminal `canup` followed by:
```bash
cd ~/Group7LitterRobot/sapling_ws
source install/setup.bash
ros2 launch piper start_single_piper.launch.py can_port:=can_arm
```
After a single instance reporting connection to be False, connection should appear to be True. Then, in a separate terminal, to launch the full retrieval system, first run:
```bash
cd ~/Group7LitterRobot/sapling_ws
source install/setup.bash
ros2 launch computer_vision vision_start.launch.py
```
Where the above will launch the litter detection subsystem. Again in a separate terminal, to activate the moveit nodes to control the arm to pick up litter, run:
```bash
cd ~/Group7LitterRobot/sapling_ws
source install/setup.bash
ros2 launch piper_with_gripper_moveit demo.launch.py 
```
which will fully set up the system. At this point, you should see the arm rise up to its default position. 

The following pubs will also require separate terminals. 

### Boolean Topic Flags

The bin_ready flag is used to only allow the litter to be dropped by PickBot in the event that the bin is in correct proximity to PickBot. As GPS was not fully implemented, this was reset to always be True, however can be configured to be False. In order to trigger the drop whilst running the "False" code, run:
```bash
ros2 topic pub --once /comms/follower_to_leader sapling_interfaces/msg/FollowerStatus "{seq: 0, parked: false, park_x: 0.0, park_y: 0.0, bin_ready: true}"
```

Similarly, as the navigation subsystem was partially implemented, grasping will only activate once a flag is given from the navigation nodes. This could be disabled via changing "REACHABILITY RADIUS and self.start_grasp" to simply "REACHABILITY RADIUS" in vision/computer_vision/computer_vision/filter_node, however grasping can be manually triggered by setting the flag as follows: 
```bash
ros2 topic pub --once /start_grasp std_msgs/msg/Bool "{data: true}"
```

The original file containing this information used during the project is located in arm/README.md

### Erroneous Behaviour

If repeated false connections occur or any other unexpected behaviour, `Ctrl+C` on all terminals to stop all nodes, and run:

candown

Where the above disables the CAN ports. Then, holding the arm about the gripper to prevent it falling, press the AC of the Bluetti battery pack to turn the arm off (ensuring the green LEDs turn off), where it should be then placed back into its resting position. Repeat the startup process to start the arm again.

### Erroneous Behaviour Warning

If the arm performs any movements which appear extremely erroneous or parts of the arm appear locked, likely after an unexpected collision, DO NOT RUN ANY FURTHER CODE. Turn off and reset the arm as with the above method, as the arm may not calculate that its joints lie in erroneous positions, and thus may collide with objects in forbidden zones and cause damage. 

Failure of the litter detection subsystem nodes to launch should be treated in the same manner as docs/Litter Detection Subsystem/Startup.md. 
