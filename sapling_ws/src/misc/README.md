# misc package

This package contains utility nodes for the Sapling robot project.

## Nodes

### sapling_buzzer_node
Provides a service to pulse a physical beeper connected to the robot.
- **Service**: `/comms/pulse_beeper` ([std_srvs/Trigger](https://docs.ros2.org/latest/api/std_srvs/srv/Trigger.html))
- **Hardware**: Uses GPIO pin D16 (board.D16).

### sapling_emergency_stop_node
Monitors both physical and virtual emergency stop triggers. When an emergency stop is active, it brings down the CAN interfaces to stop robot movement.
- **Service**: `/comms/virtual_emergency_stop` ([std_srvs/SetBool](https://docs.ros2.org/latest/api/std_srvs/srv/SetBool.html))
- **Hardware**: Monitors GPIO pin D19 (board.D19) for the physical E-stop button.
- **Behavior**:
  - Brings down `can_scout` (and `can_arm` if `SAPLING_ROLE` is `pickbot`) when triggered.
  - Automatically brings them back up when released.

**Note**: To allow the node to manage CAN interfaces without a password prompt, run `sudo visudo` and add the following line at the bottom of the file:
```bash
group7 ALL=(ALL) NOPASSWD: /usr/sbin/ip link set can_scout *, /usr/sbin/ip link set can_arm *
```

## Launch

To start all nodes in this package:
```bash
ros2 launch misc misc.launch.py
```
