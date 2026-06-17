# rover_tracking

HSV colour-based object tracking with PID-controlled pan/tilt servos.

## Topics

| Topic | Type | Direction |
|-------|------|-----------|
| `/camera/image_raw` | `sensor_msgs/Image` | Subscribed |
| `/tracking_enable` | `std_msgs/Bool` | Subscribed |
| `/pan_cmd` | `std_msgs/Float32` | Published |
| `/tilt_cmd` | `std_msgs/Float32` | Published |
| `/tracking_status` | `rover_interfaces/TrackingStatus` | Published |
| `/camera/tracked_image` | `sensor_msgs/Image` | Published |

## Tuning HSV Colour

Use the HSV tuner script:

```bash
python3 tools/hsv_tuner.py
```

Adjust sliders until the target colour is white in the mask window. Copy the values to `config/tracking.yaml`.

## Run

```bash
ros2 run rover_tracking tracking_node
```
