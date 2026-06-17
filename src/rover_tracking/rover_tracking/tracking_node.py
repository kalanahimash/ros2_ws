"""
rover_tracking — HSV object tracking node with PID servo control.

States: DISABLED → SEARCHING → TRACKING → LOST → SEARCHING

Published topics:
  /pan_cmd               (std_msgs/Float32)
  /tilt_cmd              (std_msgs/Float32)
  /tracking_status       (rover_interfaces/TrackingStatus)
  /camera/tracked_image  (sensor_msgs/Image)

Subscribed topics:
  /camera/image_raw  (sensor_msgs/Image)
  /tracking_enable   (std_msgs/Bool)
"""

from __future__ import annotations

import time
from enum import Enum, auto
from typing import Optional

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import Float32, Bool
from rover_interfaces.msg import TrackingStatus

try:
    from cv_bridge import CvBridge
    CV_BRIDGE_AVAILABLE = True
except ImportError:
    CV_BRIDGE_AVAILABLE = False

from rover_tracking.pid import PIDController
from rover_tracking.hsv_detector import HSVDetector


_CAMERA_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)

_RELIABLE_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=5,
)


class TrackingState(Enum):
    DISABLED   = auto()
    SEARCHING  = auto()
    TRACKING   = auto()
    LOST       = auto()


class TrackingNode(Node):
    """Processes camera frames and outputs pan/tilt servo commands."""

    def __init__(self) -> None:
        super().__init__("rover_tracking")

        # ── Parameters ──────────────────────────────────────────────────────
        self.declare_parameter("hsv_lower",    [5, 100, 100])
        self.declare_parameter("hsv_upper",    [25, 255, 255])
        self.declare_parameter("min_area",     500.0)
        self.declare_parameter("dead_zone",    15.0)
        self.declare_parameter("pan_kp",       0.05)
        self.declare_parameter("pan_ki",       0.001)
        self.declare_parameter("pan_kd",       0.01)
        self.declare_parameter("tilt_kp",      0.05)
        self.declare_parameter("tilt_ki",      0.001)
        self.declare_parameter("tilt_kd",      0.01)
        self.declare_parameter("servo_pan_min",   0.0)
        self.declare_parameter("servo_pan_max",   180.0)
        self.declare_parameter("servo_pan_center", 90.0)
        self.declare_parameter("servo_tilt_min",  30.0)
        self.declare_parameter("servo_tilt_max",  150.0)
        self.declare_parameter("servo_tilt_center", 90.0)
        self.declare_parameter("lost_timeout",   3.0)
        self.declare_parameter("search_speed",   0.3)

        hsv_lower  = self.get_parameter("hsv_lower").value
        hsv_upper  = self.get_parameter("hsv_upper").value
        min_area   = self.get_parameter("min_area").value
        dead_zone  = self.get_parameter("dead_zone").value

        self._pan_kp  = self.get_parameter("pan_kp").value
        self._pan_ki  = self.get_parameter("pan_ki").value
        self._pan_kd  = self.get_parameter("pan_kd").value
        self._tilt_kp = self.get_parameter("tilt_kp").value
        self._tilt_ki = self.get_parameter("tilt_ki").value
        self._tilt_kd = self.get_parameter("tilt_kd").value

        self._pan_min    = self.get_parameter("servo_pan_min").value
        self._pan_max    = self.get_parameter("servo_pan_max").value
        self._pan_center = self.get_parameter("servo_pan_center").value
        self._tilt_min   = self.get_parameter("servo_tilt_min").value
        self._tilt_max   = self.get_parameter("servo_tilt_max").value
        self._tilt_center = self.get_parameter("servo_tilt_center").value

        self._lost_timeout = self.get_parameter("lost_timeout").value
        self._search_speed = self.get_parameter("search_speed").value
        self._dead_zone    = dead_zone

        # ── State ───────────────────────────────────────────────────────────
        self._state      = TrackingState.DISABLED
        self._pan_angle  = self._pan_center
        self._tilt_angle = self._tilt_center
        self._last_seen  = 0.0
        self._search_dir = 1.0

        # ── Subsystems ──────────────────────────────────────────────────────
        self._detector = HSVDetector(
            hsv_lower=tuple(hsv_lower),
            hsv_upper=tuple(hsv_upper),
            min_area=min_area,
        )
        self._pan_pid  = PIDController(self._pan_kp,  self._pan_ki,  self._pan_kd)
        self._tilt_pid = PIDController(self._tilt_kp, self._tilt_ki, self._tilt_kd)
        self._bridge   = CvBridge() if CV_BRIDGE_AVAILABLE else None

        # ── FPS tracking ────────────────────────────────────────────────────
        self._frame_count   = 0
        self._fps_start     = time.monotonic()
        self._current_fps   = 0.0

        # ── Publishers ──────────────────────────────────────────────────────
        self._pub_pan     = self.create_publisher(Float32,        "/pan_cmd",              _RELIABLE_QOS)
        self._pub_tilt    = self.create_publisher(Float32,        "/tilt_cmd",             _RELIABLE_QOS)
        self._pub_status  = self.create_publisher(TrackingStatus, "/tracking_status",      _RELIABLE_QOS)
        self._pub_tracked = self.create_publisher(Image,          "/camera/tracked_image", _CAMERA_QOS)

        # ── Subscribers ─────────────────────────────────────────────────────
        self.create_subscription(Image, "/camera/image_raw",  self._on_image,           _CAMERA_QOS)
        self.create_subscription(Bool,  "/tracking_enable",   self._on_tracking_enable, _RELIABLE_QOS)

        self.get_logger().info("rover_tracking started (state=DISABLED)")

    # ─── Callbacks ──────────────────────────────────────────────────────────

    def _on_tracking_enable(self, msg: Bool) -> None:
        if msg.data and self._state == TrackingState.DISABLED:
            self._state = TrackingState.SEARCHING
            self._pan_pid.reset()
            self._tilt_pid.reset()
            self.get_logger().info("Tracking ENABLED → SEARCHING")
        elif not msg.data:
            self._state = TrackingState.DISABLED
            self.get_logger().info("Tracking DISABLED")
            self._publish_status(tracking=False, cx=0.0, cy=0.0, area=0.0)

    def _on_image(self, msg: Image) -> None:
        if self._state == TrackingState.DISABLED:
            return

        frame = self._decode_image(msg)
        if frame is None:
            return

        h, w = frame.shape[:2]
        frame_cx = w / 2.0
        frame_cy = h / 2.0

        detection = self._detector.detect(frame)
        self._update_fps()

        if detection is not None:
            self._handle_detection(detection, frame_cx, frame_cy)
        else:
            self._handle_no_detection()

        # Publish annotated frame
        overlay = self._detector.draw_overlay(frame, detection, frame_cx, frame_cy)
        self._pub_tracked.publish(self._encode_image(overlay, msg.header))

    # ─── State Machine ───────────────────────────────────────────────────────

    def _handle_detection(self, det, frame_cx: float, frame_cy: float) -> None:
        self._state     = TrackingState.TRACKING
        self._last_seen = time.monotonic()

        error_x = det.cx - frame_cx
        error_y = det.cy - frame_cy

        # Dead zone — suppress small corrections
        if abs(error_x) > self._dead_zone:
            pan_delta = self._pan_pid.compute(error_x)
            self._pan_angle = max(self._pan_min,
                                  min(self._pan_max, self._pan_angle + pan_delta))
        if abs(error_y) > self._dead_zone:
            tilt_delta = self._tilt_pid.compute(error_y)
            self._tilt_angle = max(self._tilt_min,
                                   min(self._tilt_max, self._tilt_angle + tilt_delta))

        self._publish_servos()
        self._publish_status(tracking=True, cx=det.cx, cy=det.cy, area=det.area)

    def _handle_no_detection(self) -> None:
        if self._state == TrackingState.TRACKING:
            self._state = TrackingState.LOST

        if self._state == TrackingState.LOST:
            if time.monotonic() - self._last_seen > self._lost_timeout:
                self._state = TrackingState.SEARCHING
                self._pan_pid.reset()
                self._tilt_pid.reset()

        if self._state == TrackingState.SEARCHING:
            self._pan_angle += self._search_dir * self._search_speed
            if self._pan_angle >= self._pan_max:
                self._pan_angle = self._pan_max
                self._search_dir = -1.0
            elif self._pan_angle <= self._pan_min:
                self._pan_angle = self._pan_min
                self._search_dir = 1.0
            self._publish_servos()

        self._publish_status(tracking=False, cx=0.0, cy=0.0, area=0.0)

    # ─── Publishing ─────────────────────────────────────────────────────────

    def _publish_servos(self) -> None:
        pan_msg  = Float32(); pan_msg.data  = float(self._pan_angle)
        tilt_msg = Float32(); tilt_msg.data = float(self._tilt_angle)
        self._pub_pan.publish(pan_msg)
        self._pub_tilt.publish(tilt_msg)

    def _publish_status(self, tracking: bool, cx: float, cy: float, area: float) -> None:
        msg = TrackingStatus()
        msg.tracking    = tracking
        msg.target_x    = cx
        msg.target_y    = cy
        msg.target_area = area
        msg.state       = self._state.name
        msg.fps         = self._current_fps
        self._pub_status.publish(msg)

    # ─── Helpers ─────────────────────────────────────────────────────────────

    def _decode_image(self, msg: Image) -> Optional[np.ndarray]:
        if self._bridge is not None:
            try:
                return self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            except Exception:
                pass
        # Manual decode
        try:
            data = np.frombuffer(bytes(msg.data), dtype=np.uint8)
            return data.reshape((msg.height, msg.width, 3))
        except Exception:
            return None

    def _encode_image(self, frame: np.ndarray, header) -> Image:
        if self._bridge is not None:
            try:
                img = self._bridge.cv2_to_imgmsg(frame, encoding="bgr8")
                img.header = header
                return img
            except Exception:
                pass
        msg = Image()
        msg.header     = header
        msg.height     = frame.shape[0]
        msg.width      = frame.shape[1]
        msg.encoding   = "bgr8"
        msg.step       = frame.shape[1] * 3
        msg.data       = frame.tobytes()
        return msg

    def _update_fps(self) -> None:
        self._frame_count += 1
        now = time.monotonic()
        elapsed = now - self._fps_start
        if elapsed >= 5.0:
            self._current_fps = self._frame_count / elapsed
            self._frame_count = 0
            self._fps_start   = now


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TrackingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
