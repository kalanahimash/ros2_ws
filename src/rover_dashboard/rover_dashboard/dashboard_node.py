"""
rover_dashboard — Flask web dashboard node.

Runs Flask + Flask-SocketIO in a background thread. The ROS2 spin loop
runs in the main thread. State is shared via a thread-safe RoverState object.

Access at: http://<raspberry-pi-ip>:5000

Published topics:
  /manual_cmd       (rover_interfaces/RoverCmd)
  /pan_cmd          (std_msgs/Float32)
  /tilt_cmd         (std_msgs/Float32)
  /tracking_enable  (std_msgs/Bool)
  /autonomous_enable(std_msgs/Bool)
  /emergency_stop   (std_msgs/Bool)

Subscribed topics:
  /camera/image_raw      (sensor_msgs/Image)   — for MJPEG stream
  /camera/tracked_image  (sensor_msgs/Image)   — when tracking active
  /distance              (std_msgs/Float32)
  /system_status         (rover_interfaces/SystemStatus)
  /pan_angle             (std_msgs/Float32)
  /tilt_angle            (std_msgs/Float32)
  /tracking_status       (rover_interfaces/TrackingStatus)
  /navigation_status     (rover_interfaces/NavigationStatus)
"""

from __future__ import annotations

import io
import os
import threading
import time
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import Float32, Bool
from rover_interfaces.msg import RoverCmd, SystemStatus, TrackingStatus, NavigationStatus

try:
    import cv2
    import numpy as np
    CV2_AVAILABLE = True
except ImportError:
    CV2_AVAILABLE = False

try:
    from cv_bridge import CvBridge
    CV_BRIDGE_AVAILABLE = True
except ImportError:
    CV_BRIDGE_AVAILABLE = False

try:
    from flask import Flask, Response, jsonify, request, render_template
    from flask_socketio import SocketIO, emit
    FLASK_AVAILABLE = True
except ImportError:
    FLASK_AVAILABLE = False

from rover_dashboard.state import RoverState


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

_ESTOP_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)


def _find_templates_dir() -> str:
    """Locate the templates directory relative to this file."""
    candidates = [
        os.path.join(os.path.dirname(__file__), "templates"),
        os.path.join(os.path.dirname(__file__), "..", "templates"),
    ]
    for c in candidates:
        if os.path.isdir(c):
            return os.path.abspath(c)
    return os.path.join(os.path.dirname(__file__), "templates")


def _find_static_dir() -> str:
    candidates = [
        os.path.join(os.path.dirname(__file__), "static"),
        os.path.join(os.path.dirname(__file__), "..", "static"),
    ]
    for c in candidates:
        if os.path.isdir(c):
            return os.path.abspath(c)
    return os.path.join(os.path.dirname(__file__), "static")


class DashboardNode(Node):
    """ROS2 node hosting the Flask dashboard."""

    def __init__(self) -> None:
        super().__init__("rover_dashboard")

        self.declare_parameter("host",       "0.0.0.0")
        self.declare_parameter("port",       5000)
        self.declare_parameter("debug",      False)
        self.declare_parameter("stream_quality", 85)

        self._host    = self.get_parameter("host").value
        self._port    = self.get_parameter("port").value
        self._quality = self.get_parameter("stream_quality").value

        self._state  = RoverState()
        self._bridge = CvBridge() if CV_BRIDGE_AVAILABLE else None

        # ── Publishers ──────────────────────────────────────────────────────
        self._pub_manual   = self.create_publisher(RoverCmd, "/manual_cmd",       _RELIABLE_QOS)
        self._pub_pan      = self.create_publisher(Float32,  "/pan_cmd",          _RELIABLE_QOS)
        self._pub_tilt     = self.create_publisher(Float32,  "/tilt_cmd",         _RELIABLE_QOS)
        self._pub_tracking = self.create_publisher(Bool,     "/tracking_enable",  _RELIABLE_QOS)
        self._pub_auto     = self.create_publisher(Bool,     "/autonomous_enable", _RELIABLE_QOS)
        self._pub_estop    = self.create_publisher(Bool,     "/emergency_stop",   _ESTOP_QOS)

        # ── Subscribers ─────────────────────────────────────────────────────
        self.create_subscription(Image,           "/camera/tracked_image", self._on_image,   _CAMERA_QOS)
        self.create_subscription(Image,           "/camera/image_raw",     self._on_image_raw, _CAMERA_QOS)
        self.create_subscription(Float32,         "/distance",             self._on_distance, _RELIABLE_QOS)
        self.create_subscription(SystemStatus,    "/system_status",        self._on_status,  _RELIABLE_QOS)
        self.create_subscription(Float32,         "/pan_angle",            self._on_pan,     _RELIABLE_QOS)
        self.create_subscription(Float32,         "/tilt_angle",           self._on_tilt,    _RELIABLE_QOS)
        self.create_subscription(TrackingStatus,  "/tracking_status",      self._on_tracking,_RELIABLE_QOS)
        self.create_subscription(NavigationStatus,"/navigation_status",    self._on_nav,     _RELIABLE_QOS)

        # ── SocketIO push timer ─────────────────────────────────────────────
        self._socketio: Optional[SocketIO] = None
        self._telemetry_timer = self.create_timer(0.1, self._push_telemetry)

        # ── Start Flask in background ────────────────────────────────────────
        self._flask_thread = threading.Thread(target=self._run_flask, daemon=True)
        self._flask_thread.start()

        self.get_logger().info(
            f"rover_dashboard started → http://{self._host}:{self._port}"
        )

    # ─── ROS2 Subscriber Callbacks ───────────────────────────────────────────

    def _on_image(self, msg: Image) -> None:
        jpeg = self._image_to_jpeg(msg)
        if jpeg:
            with self._state.jpeg_lock:
                self._state.latest_jpeg = jpeg

    def _on_image_raw(self, msg: Image) -> None:
        # Only use raw image if no tracked image available
        with self._state.jpeg_lock:
            if self._state.latest_jpeg is None:
                jpeg = self._image_to_jpeg(msg)
                if jpeg:
                    self._state.latest_jpeg = jpeg

    def _on_distance(self, msg: Float32) -> None:
        with self._state.lock:
            self._state.distance = msg.data
            self._state.serial_connected = True

    def _on_status(self, msg: SystemStatus) -> None:
        with self._state.lock:
            self._state.mode             = msg.mode
            self._state.battery_voltage  = msg.battery_voltage
            self._state.motors_enabled   = msg.motors_enabled
            self._state.watchdog_ok      = msg.watchdog_ok

    def _on_pan(self, msg: Float32) -> None:
        with self._state.lock:
            self._state.pan_angle = msg.data

    def _on_tilt(self, msg: Float32) -> None:
        with self._state.lock:
            self._state.tilt_angle = msg.data

    def _on_tracking(self, msg: TrackingStatus) -> None:
        with self._state.lock:
            self._state.tracking       = msg.tracking
            self._state.tracking_state = msg.state
            self._state.tracking_fps   = msg.fps

    def _on_nav(self, msg: NavigationStatus) -> None:
        with self._state.lock:
            self._state.autonomous        = msg.autonomous_enabled
            self._state.nav_state         = msg.state
            self._state.obstacle_detected = msg.obstacle_detected

    # ─── SocketIO Telemetry Push ─────────────────────────────────────────────

    def _push_telemetry(self) -> None:
        if self._socketio is None:
            return
        try:
            self._socketio.emit("status", self._state.to_dict(), namespace="/")
        except Exception:
            pass

    # ─── Image Processing ────────────────────────────────────────────────────

    def _image_to_jpeg(self, msg: Image) -> Optional[bytes]:
        if not CV2_AVAILABLE:
            return None
        try:
            if self._bridge is not None:
                frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            else:
                data  = np.frombuffer(bytes(msg.data), dtype=np.uint8)
                frame = data.reshape((msg.height, msg.width, 3))
            _, buf = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, self._quality])
            return bytes(buf)
        except Exception:
            return None

    # ─── Flask App ───────────────────────────────────────────────────────────

    def _run_flask(self) -> None:
        if not FLASK_AVAILABLE:
            self.get_logger().error("Flask/Flask-SocketIO not installed — dashboard unavailable")
            return

        templates = _find_templates_dir()
        static    = _find_static_dir()

        app = Flask(__name__, template_folder=templates, static_folder=static)
        app.config["SECRET_KEY"] = "rover-secret-key"
        socketio = SocketIO(app, cors_allowed_origins="*", async_mode="threading")
        self._socketio = socketio

        node = self  # closure reference

        # ── Routes ──────────────────────────────────────────────────────────

        @app.route("/")
        def index():
            return render_template("index.html")

        @app.route("/video_feed")
        def video_feed():
            def generate():
                while True:
                    with node._state.jpeg_lock:
                        jpeg = node._state.latest_jpeg
                    if jpeg:
                        yield (
                            b"--frame\r\n"
                            b"Content-Type: image/jpeg\r\n\r\n" + jpeg + b"\r\n"
                        )
                    else:
                        # Black placeholder frame
                        placeholder = _black_jpeg()
                        yield (
                            b"--frame\r\n"
                            b"Content-Type: image/jpeg\r\n\r\n" + placeholder + b"\r\n"
                        )
                    time.sleep(0.033)

            return Response(
                generate(),
                mimetype="multipart/x-mixed-replace; boundary=frame"
            )

        @app.route("/api/status")
        def api_status():
            return jsonify(node._state.to_dict())

        @app.route("/api/cmd", methods=["POST"])
        def api_cmd():
            data    = request.get_json(force=True) or {}
            linear  = float(data.get("linear",  0.0))
            angular = float(data.get("angular", 0.0))
            msg = RoverCmd()
            msg.linear  = max(-1.0, min(1.0, linear))
            msg.angular = max(-1.0, min(1.0, angular))
            node._pub_manual.publish(msg)
            return jsonify({"ok": True})

        @app.route("/api/pan", methods=["POST"])
        def api_pan():
            data = request.get_json(force=True) or {}
            angle = float(data.get("angle", 90.0))
            msg = Float32(); msg.data = angle
            node._pub_pan.publish(msg)
            return jsonify({"ok": True})

        @app.route("/api/tilt", methods=["POST"])
        def api_tilt():
            data = request.get_json(force=True) or {}
            angle = float(data.get("angle", 90.0))
            msg = Float32(); msg.data = angle
            node._pub_tilt.publish(msg)
            return jsonify({"ok": True})

        @app.route("/api/tracking", methods=["POST"])
        def api_tracking():
            data    = request.get_json(force=True) or {}
            enabled = bool(data.get("enabled", False))
            msg = Bool(); msg.data = enabled
            node._pub_tracking.publish(msg)
            return jsonify({"ok": True})

        @app.route("/api/autonomous", methods=["POST"])
        def api_autonomous():
            data    = request.get_json(force=True) or {}
            enabled = bool(data.get("enabled", False))
            msg = Bool(); msg.data = enabled
            node._pub_auto.publish(msg)
            return jsonify({"ok": True})

        @app.route("/api/estop", methods=["POST"])
        def api_estop():
            msg = Bool(); msg.data = True
            node._pub_estop.publish(msg)
            with node._state.lock:
                node._state.estop_active = True
            return jsonify({"ok": True})

        @app.route("/api/reset", methods=["POST"])
        def api_reset():
            msg = Bool(); msg.data = False
            node._pub_estop.publish(msg)
            with node._state.lock:
                node._state.estop_active = False
            return jsonify({"ok": True})

        # ── SocketIO Events ─────────────────────────────────────────────────

        @socketio.on("joystick")
        def on_joystick(data):
            linear  = float(data.get("linear",  0.0))
            angular = float(data.get("angular", 0.0))
            cmd = RoverCmd()
            cmd.linear  = max(-1.0, min(1.0, linear))
            cmd.angular = max(-1.0, min(1.0, angular))
            node._pub_manual.publish(cmd)

        @socketio.on("estop")
        def on_estop(_data=None):
            msg = Bool(); msg.data = True
            node._pub_estop.publish(msg)
            with node._state.lock:
                node._state.estop_active = True

        @socketio.on("reset")
        def on_reset(_data=None):
            msg = Bool(); msg.data = False
            node._pub_estop.publish(msg)
            with node._state.lock:
                node._state.estop_active = False

        # ── Run ─────────────────────────────────────────────────────────────
        socketio.run(
            app,
            host=self._host,
            port=self._port,
            use_reloader=False,
            log_output=False,
            allow_unsafe_werkzeug=True,
        )


def _black_jpeg() -> bytes:
    """Return a minimal JPEG for when no camera frame is available."""
    if CV2_AVAILABLE:
        import numpy as np
        black = np.zeros((240, 320, 3), dtype=np.uint8)
        _, buf = cv2.imencode(".jpg", black)
        return bytes(buf)
    # Minimal 1×1 black JPEG (hardcoded)
    return (
        b"\xff\xd8\xff\xe0\x00\x10JFIF\x00\x01\x01\x00\x00\x01\x00\x01\x00\x00"
        b"\xff\xdb\x00C\x00\x08\x06\x06\x07\x06\x05\x08\x07\x07\x07\t\t"
        b"\x08\n\x0c\x14\r\x0c\x0b\x0b\x0c\x19\x12\x13\x0f\x14\x1d\x1a"
        b"\x1f\x1e\x1d\x1a\x1c\x1c $.' \",#\x1c\x1c(7),01444\x1f'9=82<.342\x1e"
        b"\xff\xc0\x00\x0b\x08\x00\x01\x00\x01\x01\x01\x11\x00\xff\xc4\x00\x1f"
        b"\x00\x00\x01\x05\x01\x01\x01\x01\x01\x01\x00\x00\x00\x00\x00\x00\x00"
        b"\x00\x01\x02\x03\x04\x05\x06\x07\x08\t\n\x0b\xff\xda\x00\x08\x01\x01"
        b"\x00\x00?\x00\xfb\xd7\xff\xd9"
    )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DashboardNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
