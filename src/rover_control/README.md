# rover_control

Command arbitration node. Merges manual (dashboard) and autonomous (navigation) commands with priority and safety enforcement.

## Priority

1. Emergency stop (latching) — overrides all
2. Manual command — active for `manual_timeout` seconds after last received
3. Autonomous command

## Topics

| Topic | Type | Direction |
|-------|------|-----------|
| `/manual_cmd` | `rover_interfaces/RoverCmd` | Subscribed |
| `/auto_cmd` | `rover_interfaces/RoverCmd` | Subscribed |
| `/emergency_stop` | `std_msgs/Bool` | Subscribed |
| `/rover_cmd` | `rover_interfaces/RoverCmd` | Published |

## Run

```bash
ros2 run rover_control control_node
```
