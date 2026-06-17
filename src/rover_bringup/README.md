# rover_bringup

Launch files and system-wide configuration for the autonomous rover.

## Launch Files

| File | Description |
|------|-------------|
| `rover.launch.py` | Full system bringup (all 6 nodes) |
| `serial_only.launch.py` | Serial + control only (hardware testing) |

## Usage

```bash
# Full system
ros2 launch rover_bringup rover.launch.py

# Override serial port
ros2 launch rover_bringup rover.launch.py serial_port:=/dev/ttyACM0

# Override dashboard port
ros2 launch rover_bringup rover.launch.py dashboard_port:=8080

# Serial only (hardware test)
ros2 launch rover_bringup serial_only.launch.py
```

## Configuration

Edit `config/rover_params.yaml` to tune all parameters without recompiling.
