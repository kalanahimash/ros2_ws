# rover_camera

Camera capture node. Supports Picamera2 (Raspberry Pi), OpenCV USB webcam, and synthetic test frames.

## Published Topics

| Topic | Type | Rate |
|-------|------|------|
| `/camera/image_raw` | `sensor_msgs/Image` | 30 Hz |
| `/camera/camera_info` | `sensor_msgs/CameraInfo` | 30 Hz |

## Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `width` | `640` | Frame width |
| `height` | `480` | Frame height |
| `fps` | `30` | Target frame rate |
| `frame_id` | `camera_link` | TF frame |
| `device_index` | `0` | OpenCV device index |

## Run

```bash
ros2 run rover_camera camera_node
```
