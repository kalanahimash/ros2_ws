# rover_navigation

FSM-based obstacle avoidance. Drives forward until it detects an obstacle, then reverses and turns.

## Topics

| Topic | Type | Direction |
|-------|------|-----------|
| `/distance` | `std_msgs/Float32` | Subscribed |
| `/autonomous_enable` | `std_msgs/Bool` | Subscribed |
| `/emergency_stop` | `std_msgs/Bool` | Subscribed |
| `/auto_cmd` | `rover_interfaces/RoverCmd` | Published |
| `/navigation_status` | `rover_interfaces/NavigationStatus` | Published |

## States

`IDLE → FORWARD → STOP → REVERSE → TURN_LEFT|RIGHT → FORWARD`

Emergency stop from any state when `/emergency_stop=True` or `distance < 10 cm`.

## Run

```bash
ros2 run rover_navigation navigation_node
```
