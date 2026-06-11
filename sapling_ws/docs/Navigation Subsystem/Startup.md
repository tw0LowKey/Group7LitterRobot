### Final Demo Command Sequence

For the final integrated demonstration, the commands are run in the following order.

Terminal 1:

```bash
canup
```

Terminal 2:

```bash
cd ~/Group7LitterRobot/sapling_ws
source install/setup.bash
ros2 launch computer_vision vision_start.launch.py
```

Terminal 3:

```bash
cd ~/Group7LitterRobot/sapling_ws
source install/setup.bash
ros2 launch piper start_single_piper.launch.py can_port:=can_arm
```

Terminal 4:

```bash
cd ~/Group7LitterRobot/sapling_ws
source install/setup.bash
ros2 launch piper_with_gripper_moveit demo.launch.py
```

Terminal 5:

```bash
cd ~/Group7LitterRobot/sapling_ws
source install/setup.bash
ros2 launch rm_navigation full_indoor_navigation.launch.py
```

Terminal 6, to start the navigation behaviour, after 30s initial cooldown:

```bash
ros2 topic pub --once /drive_front_3m std_msgs/msg/Bool "{data: true}"
```

If the grasping stage needs to be triggered manually:

```bash
ros2 topic pub --once /start_grasp std_msgs/msg/Bool "{data: true}"
```
