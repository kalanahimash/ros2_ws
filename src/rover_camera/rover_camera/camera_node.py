"""
rover_camera — Camera capture and image publishing node.

Supports Picamera2 (Raspberry Pi) with automatic fallback to OpenCV
(USB webcam or test environment). Publishes BGR8 images at the
configured frame rate.

Published topics:
  /camera/image_raw   (sensor_msgs/Image)
  /camera/camera_info (sensor_msgs/CameraInfo)
"""

from __future__ import annotations

import time
from typing import Optional

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image, CameraInfo
from builtin_interfaces.msg import Time

try:
    from cv_bridge import CvBridge
    CV_BRIDGE_AVAILABLE = True
except ImportError:
    CV_BRIDGE_AVAILABLE = False

# Try Picamera2 first (Raspberry Pi), fall back to OpenCV
try:
    from picamera2 import Picamera2
    PICAMERA2_AVAILABLE = True
except ImportError:
    PICAMERA2_AVAILABLE = False

try:
    import cv2
    CV2_AVAILABLE = True
except ImportError:
    CV2_AVAILABLE = False


_CAMERA_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)


class CameraNode(Node):
    """Captures frames and publishes them as ROS2 Image messages."""

    def __init__(self) -> None:
        super().__init__("rover_camera")

        self.declare_parameter("width",    640)
        self.declare_parameter("height",   480)
        self.declare_parameter("fps",      30)
        self.declare_parameter("frame_id", "camera_link")
        self.declare_parameter("device_index", 0)

        self._width    = self.get_parameter("width").value
        self._height   = self.get_parameter("height").value
        self._fps      = self.get_parameter("fps").value
        self._frame_id = self.get_parameter("frame_id").value
        self._device   = self.get_parameter("device_index").value

        self._bridge  = CvBridge() if CV_BRIDGE_AVAILABLE else None
        self._camera  = None
        self._backend = "none"

        self._pub_image = self.create_publisher(Image,      "/camera/image_raw",   _CAMERA_QOS)
        self._pub_info  = self.create_publisher(CameraInfo, "/camera/camera_info", _CAMERA_QOS)

        self._frame_count   = 0
        self._fps_last_time = time.time()
        self._actual_fps    = 0.0

        self._init_camera()

        period = 1.0 / self._fps
        self._timer = self.create_timer(period, self._capture_and_publish)

        self.get_logger().info(
            f"rover_camera started: {self._width}×{self._height} @ {self._fps} FPS "
            f"(backend={self._backend})"
        )

    # ─── Camera Initialisation ──────────────────────────────────────────────

    def _init_camera(self) -> None:
        if PICAMERA2_AVAILABLE:
            try:
                self._camera = Picamera2()
                config = self._camera.create_video_configuration(
                    main={"size": (self._width, self._height), "format": "BGR888"}
                )
                self._camera.configure(config)
                self._camera.start()
                self._backend = "picamera2"
                self.get_logger().info("Using Picamera2 backend")
                return
            except Exception as exc:
                self.get_logger().warn(f"Picamera2 init failed: {exc}")

        if CV2_AVAILABLE:
            cap = cv2.VideoCapture(self._device)
            if cap.isOpened():
                cap.set(cv2.CAP_PROP_FRAME_WIDTH,  self._width)
                cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self._height)
                cap.set(cv2.CAP_PROP_FPS,          self._fps)
                self._camera = cap
                self._backend = "opencv"
                self.get_logger().info(f"Using OpenCV backend (device {self._device})")
                return
            cap.release()

        self._backend = "synthetic"
        self.get_logger().warn(
            "No camera available — publishing synthetic test frames"
        )

    # ─── Frame Capture ──────────────────────────────────────────────────────

    def _capture_frame(self) -> Optional[np.ndarray]:
        if self._backend == "picamera2":
            try:
                return self._camera.capture_array()
            except Exception as exc:
                self.get_logger().warn(f"Picamera2 capture error: {exc}")
                self._recover()
                return None

        if self._backend == "opencv":
            ret, frame = self._camera.read()
            if not ret:
                self.get_logger().warn("OpenCV frame capture failed")
                self._recover()
                return None
            return frame

        # Synthetic: colourful test pattern
        frame = np.zeros((self._height, self._width, 3), dtype=np.uint8)
        t = int(time.time() * 30) % self._width
        frame[:, :, 0] = 40
        frame[:, :, 1] = 40
        frame[:, :, 2] = 40
        frame[:, t:t+20, :] = [0, 200, 255]  # moving yellow bar
        return frame

    def _recover(self) -> None:
        self.get_logger().warn("Attempting camera recovery...")
        try:
            if self._backend == "picamera2" and self._camera:
                self._camera.stop()
                self._camera.start()
            elif self._backend == "opencv" and self._camera:
                self._camera.release()
                self._init_camera()
        except Exception as exc:
            self.get_logger().error(f"Camera recovery failed: {exc}")
            self._backend = "synthetic"

    # ─── Publish ────────────────────────────────────────────────────────────

    def _capture_and_publish(self) -> None:
        frame = self._capture_frame()
        if frame is None:
            return

        stamp = self.get_clock().now().to_msg()

        img_msg = self._frame_to_msg(frame, stamp)
        self._pub_image.publish(img_msg)

        info_msg = self._make_camera_info(stamp)
        self._pub_info.publish(info_msg)

        self._frame_count += 1
        now = time.time()
        elapsed = now - self._fps_last_time
        if elapsed >= 5.0:
            self._actual_fps = self._frame_count / elapsed
            self._frame_count = 0
            self._fps_last_time = now
            self.get_logger().debug(f"Camera FPS: {self._actual_fps:.1f}")

    def _frame_to_msg(self, frame: np.ndarray, stamp: Time) -> Image:
        if self._bridge is not None:
            msg = self._bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            msg.header.stamp    = stamp
            msg.header.frame_id = self._frame_id
            return msg

        # Manual encoding without cv_bridge
        msg = Image()
        msg.header.stamp    = stamp
        msg.header.frame_id = self._frame_id
        msg.height          = frame.shape[0]
        msg.width           = frame.shape[1]
        msg.encoding        = "bgr8"
        msg.is_bigendian    = 0
        msg.step            = frame.shape[1] * 3
        msg.data            = frame.tobytes()
        return msg

    def _make_camera_info(self, stamp: Time) -> CameraInfo:
        info = CameraInfo()
        info.header.stamp    = stamp
        info.header.frame_id = self._frame_id
        info.width           = self._width
        info.height          = self._height
        # Simple pinhole model — calibrate for accurate measurements
        fx = fy = float(self._width)
        cx = self._width  / 2.0
        cy = self._height / 2.0
        info.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
        info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        info.p = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
        return info

    # ─── Shutdown ───────────────────────────────────────────────────────────

    def destroy_node(self) -> None:
        try:
            if self._backend == "picamera2" and self._camera:
                self._camera.stop()
            elif self._backend == "opencv" and self._camera:
                self._camera.release()
        except Exception:
            pass
        super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
