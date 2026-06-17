# rover_serial

USB serial communication between Raspberry Pi and Arduino.

## Topics

| Topic | Type | Direction |
|-------|------|-----------|
| `/distance` | `std_msgs/Float32` | Published |
| `/system_status` | `rover_interfaces/SystemStatus` | Published |
| `/pan_angle` | `std_msgs/Float32` | Published |
| `/tilt_angle` | `std_msgs/Float32` | Published |
| `/rover_cmd` | `rover_interfaces/RoverCmd` | Subscribed |
| `/pan_cmd` | `std_msgs/Float32` | Subscribed |
| `/tilt_cmd` | `std_msgs/Float32` | Subscribed |

## Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `serial_port` | `/dev/ttyUSB0` | Arduino USB port |
| `baud_rate` | `115200` | Baud rate |
| `reconnect_interval` | `3.0` | Retry interval (s) |
| `heartbeat_interval` | `1.0` | HB send interval (s) |
| `timeout` | `2.0` | Serial read timeout (s) |

## Run

```bash
ros2 run rover_serial serial_node
```

Override port:

```bash
ros2 run rover_serial serial_node --ros-args -p serial_port:=/dev/ttyACM0
```
