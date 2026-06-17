# rover_dashboard

Flask web dashboard with live MJPEG stream, WebSocket telemetry, and manual controls.

## Access

```
http://<raspberry-pi-ip>:5000
```

## Features

- Live camera stream (MJPEG)
- Real-time telemetry via WebSocket (10 Hz)
- Virtual joystick with mouse and touch support
- Keyboard shortcuts (W/A/S/D, Space, E, R, T, N)
- Pan/tilt manual sliders
- Tracking enable/disable toggle
- Autonomous mode toggle
- Emergency stop button
- Event log panel
- Responsive dark UI

## Topics

See [06_ROS2_Packages.md](../../../../docs/06_ROS2_Packages.md) for full topic list.

## Run

```bash
ros2 run rover_dashboard dashboard_node
```
