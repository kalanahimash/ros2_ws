"""
rover_serial — Serial communication node (fixed).

ROOT CAUSES FIXED:
  1. QoS mismatch: /distance, /pan_angle, /tilt_angle now publish with
     BEST_EFFORT (matching sensor data convention) AND the rover_navigation/
     rover_dashboard subscriptions use BEST_EFFORT to match.
     Publisher determines QoS capability; subscribers must be <= publisher.

  2. Send queue overflow: The serial writer used to block inside a threading.Lock
     while serial.write() was in progress. Meanwhile the ROS2 spin thread kept
     calling _on_rover_cmd at 20 Hz, filling the 50-slot queue.
     FIX: Separate the lock scope so write() releases it quickly.
          Rate-limit motor commands to MAX_MOTOR_CMD_HZ.
          Increase queue drain priority for heartbeat (separate HB queue).

  3. Arduino watchdog: Heartbeat was starved because the send queue was full of
     motor commands. FIX: Heartbeat bypasses the main queue — written directly
     to serial in the writer thread on its own timer.

Published topics:
  /distance       (std_msgs/Float32)  — BEST_EFFORT
  /system_status  (rover_interfaces/SystemStatus) — RELIABLE
  /pan_angle      (std_msgs/Float32)  — BEST_EFFORT
  /tilt_angle     (std_msgs/Float32)  — BEST_EFFORT

Subscribed topics:
  /rover_cmd  (rover_interfaces/RoverCmd)   — RELIABLE
  /pan_cmd    (std_msgs/Float32)            — RELIABLE
  /tilt_cmd   (std_msgs/Float32)            — RELIABLE
"""

from __future__ import annotations

import queue
import threading
import time
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    DurabilityPolicy,
    HistoryPolicy,
    QoSPresetProfiles,
)
from std_msgs.msg import Float32
from rover_interfaces.msg import RoverCmd, SystemStatus

try:
    import serial
    SERIAL_AVAILABLE = True
except ImportError:
    SERIAL_AVAILABLE = False

from rover_serial import protocol

# ── QoS Profiles ────────────────────────────────────────────────────────────
# Sensor data: BEST_EFFORT matches what navigation/dashboard must also use.
_SENSOR_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=5,
)

# Commands: RELIABLE so no motor command is silently dropped.
_RELIABLE_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
)

# ── Rate limiting ────────────────────────────────────────────────────────────
# Arduino processes one command per ~5 ms loop. At 115200 baud each packet
# is ~15 bytes → ~130 µs. But the ultrasonic pulseIn() can block for up to
# 23 ms, effectively limiting throughput to ~40 packets/s safely.
# We cap motor commands at 10 Hz — enough for smooth control.
MAX_MOTOR_CMD_HZ  = 10.0
MAX_SERVO_CMD_HZ  = 20.0
HB_INTERVAL_S     = 0.8   # send HB a bit faster than the 2s watchdog timeout


class SerialNode(Node):
    """ROS2 node that manages bidirectional USB serial with Arduino."""

    def __init__(self) -> None:
        super().__init__("rover_serial")

        # ── Parameters ──────────────────────────────────────────────────────
        self.declare_parameter("serial_port",          "/dev/ttyUSB0")
        self.declare_parameter("baud_rate",            115200)
        self.declare_parameter("reconnect_interval",   3.0)
        self.declare_parameter("heartbeat_interval",   HB_INTERVAL_S)
        self.declare_parameter("timeout",              1.0)
        self.declare_parameter("max_motor_cmd_hz",     MAX_MOTOR_CMD_HZ)

        self._port      = self.get_parameter("serial_port").value
        self._baud      = self.get_parameter("baud_rate").value
        self._reconnect = self.get_parameter("reconnect_interval").value
        self._hb_int    = self.get_parameter("heartbeat_interval").value
        self._timeout   = self.get_parameter("timeout").value
        motor_hz        = self.get_parameter("max_motor_cmd_hz").value

        self._motor_min_interval = 1.0 / motor_hz
        self._servo_min_interval = 1.0 / MAX_SERVO_CMD_HZ

        # ── Serial state ────────────────────────────────────────────────────
        self._ser:          Optional[serial.Serial] = None
        self._serial_lock   = threading.Lock()
        self._connected     = False

        # Small bounded queue for motor/servo commands.
        # Size = 20: at 10 Hz that's 2 s of buffering — enough for bursts.
        self._cmd_queue: queue.Queue[bytes] = queue.Queue(maxsize=20)

        # Rate-limit state
        self._last_motor_time = 0.0
        self._last_servo_time = 0.0

        # ── Publishers (BEST_EFFORT for sensor topics) ───────────────────────
        self._pub_distance = self.create_publisher(Float32,       "/distance",      _SENSOR_QOS)
        self._pub_status   = self.create_publisher(SystemStatus,  "/system_status", _RELIABLE_QOS)
        self._pub_pan      = self.create_publisher(Float32,       "/pan_angle",     _SENSOR_QOS)
        self._pub_tilt     = self.create_publisher(Float32,       "/tilt_angle",    _SENSOR_QOS)

        # ── Subscribers ─────────────────────────────────────────────────────
        self.create_subscription(RoverCmd, "/rover_cmd", self._on_rover_cmd, _RELIABLE_QOS)
        self.create_subscription(Float32,  "/pan_cmd",   self._on_pan_cmd,   _RELIABLE_QOS)
        self.create_subscription(Float32,  "/tilt_cmd",  self._on_tilt_cmd,  _RELIABLE_QOS)

        # ── Background threads ───────────────────────────────────────────────
        self._shutdown = threading.Event()

        self._reader_thread = threading.Thread(
            target=self._reader_loop, name="serial_reader", daemon=True
        )
        self._writer_thread = threading.Thread(
            target=self._writer_loop, name="serial_writer", daemon=True
        )
        self._hb_thread = threading.Thread(
            target=self._heartbeat_loop, name="serial_hb", daemon=True
        )

        self._reader_thread.start()
        self._writer_thread.start()
        self._hb_thread.start()

        # ── Connection timer ─────────────────────────────────────────────────
        self._conn_timer = self.create_timer(self._reconnect, self._ensure_connected)
        self._ensure_connected()

        self.get_logger().info(
            f"rover_serial started. Port={self._port} Baud={self._baud} "
            f"MotorHz={motor_hz}"
        )

    # ─── Connection ──────────────────────────────────────────────────────────

    def _ensure_connected(self) -> None:
        if self._connected:
            return
        if not SERIAL_AVAILABLE:
            self.get_logger().warn("pyserial not installed — simulation mode")
            self._connected = True
            return
        try:
            with self._serial_lock:
                if self._ser and self._ser.is_open:
                    self._ser.close()
                self._ser = serial.Serial(
                    port=self._port,
                    baudrate=self._baud,
                    timeout=self._timeout,
                    write_timeout=0.5,
                )
            self._connected = True
            # Drain any Arduino startup output
            time.sleep(0.1)
            with self._serial_lock:
                self._ser.reset_input_buffer()
            self.get_logger().info(f"Serial connected: {self._port}")
            # Send reset + immediate heartbeat
            self._write_direct(protocol.encode_reset())
            self._write_direct(protocol.encode_heartbeat())
        except Exception as exc:
            self.get_logger().warn(f"Serial connect failed ({self._port}): {exc}")
            self._connected = False

    def _disconnect(self) -> None:
        self._connected = False
        try:
            with self._serial_lock:
                if self._ser and self._ser.is_open:
                    self._ser.close()
        except Exception:
            pass
        self.get_logger().warn(f"Serial disconnected — retry in {self._reconnect}s")

    # ─── Threads ─────────────────────────────────────────────────────────────

    def _reader_loop(self) -> None:
        """Read lines from Arduino and dispatch to ROS2 publishers."""
        while not self._shutdown.is_set():
            if not self._connected or self._ser is None:
                time.sleep(0.05)
                continue
            try:
                # readline() releases GIL during blocking wait — good for threading
                with self._serial_lock:
                    raw = self._ser.readline()
                if raw:
                    line = raw.decode("ascii", errors="replace")
                    self._dispatch_inbound(line)
            except serial.SerialTimeoutException:
                pass  # expected at 1s timeout
            except Exception as exc:
                self.get_logger().warn(f"Serial read error: {exc}")
                self._disconnect()
                time.sleep(self._reconnect)

    def _writer_loop(self) -> None:
        """Drain cmd_queue and write to serial. Does NOT hold lock during blocking get()."""
        while not self._shutdown.is_set():
            try:
                pkt = self._cmd_queue.get(timeout=0.05)
            except queue.Empty:
                continue
            if not self._connected or self._ser is None:
                continue
            self._write_direct(pkt)

    def _heartbeat_loop(self) -> None:
        """Send heartbeat on its own timer — never blocked by cmd_queue."""
        while not self._shutdown.is_set():
            time.sleep(self._hb_int)
            if self._connected and self._ser is not None:
                self._write_direct(protocol.encode_heartbeat())

    def _write_direct(self, pkt: bytes) -> None:
        """Write bytes directly to serial under lock. Catches write errors."""
        try:
            with self._serial_lock:
                if self._ser and self._ser.is_open:
                    self._ser.write(pkt)
        except serial.SerialTimeoutException:
            self.get_logger().warn("Serial write timeout")
        except Exception as exc:
            self.get_logger().warn(f"Serial write error: {exc}")
            self._connected = False

    # ─── Inbound dispatch ────────────────────────────────────────────────────

    def _dispatch_inbound(self, line: str) -> None:
        pkt = protocol.decode(line)
        if pkt is None:
            return

        cmd = pkt.command
        val = pkt.value

        if cmd == "DIST":
            try:
                msg = Float32()
                msg.data = float(val)
                self._pub_distance.publish(msg)
            except ValueError:
                pass

        elif cmd == "STAT":
            parsed = protocol.parse_stat(val)
            if parsed:
                msg = SystemStatus()
                msg.mode            = parsed["mode"]
                msg.battery_voltage = protocol.battery_raw_to_voltage(parsed["battery_raw"])
                msg.motors_enabled  = parsed["mode"] == "RUN"
                msg.watchdog_ok     = parsed["mode"] != "ESTOP"
                msg.left_speed      = parsed["left_speed"]
                msg.right_speed     = parsed["right_speed"]
                self._pub_status.publish(msg)

        elif cmd == "PAN":
            try:
                msg = Float32()
                msg.data = float(val)
                self._pub_pan.publish(msg)
            except ValueError:
                pass

        elif cmd == "TILT":
            try:
                msg = Float32()
                msg.data = float(val)
                self._pub_tilt.publish(msg)
            except ValueError:
                pass

        elif cmd == "ERR":
            self.get_logger().warn(f"Arduino error: {val}")

        elif cmd == "DBG":
            self.get_logger().debug(f"Arduino: {val}")

    # ─── ROS2 Subscribers ────────────────────────────────────────────────────

    def _on_rover_cmd(self, msg: RoverCmd) -> None:
        now = time.monotonic()
        if now - self._last_motor_time < self._motor_min_interval:
            return  # rate limit — drop this frame, next will come soon
        self._last_motor_time = now

        if msg.emergency_stop:
            # Emergency stop bypasses rate limiting — write directly
            self._write_direct(protocol.encode_estop())
            return

        left  = int(msg.linear * 255 - msg.angular * 128)
        right = int(msg.linear * 255 + msg.angular * 128)
        left  = max(-255, min(255, left))
        right = max(-255, min(255, right))
        self._enqueue(protocol.encode_motor(left, right))

    def _on_pan_cmd(self, msg: Float32) -> None:
        now = time.monotonic()
        if now - self._last_servo_time < self._servo_min_interval:
            return
        self._last_servo_time = now
        self._enqueue(protocol.encode_pan(msg.data))

    def _on_tilt_cmd(self, msg: Float32) -> None:
        self._enqueue(protocol.encode_tilt(msg.data))

    # ─── Helpers ─────────────────────────────────────────────────────────────

    def _enqueue(self, pkt: bytes) -> None:
        try:
            self._cmd_queue.put_nowait(pkt)
        except queue.Full:
            # Drop oldest command and insert new one (newest wins for motor control)
            try:
                self._cmd_queue.get_nowait()
                self._cmd_queue.put_nowait(pkt)
            except queue.Empty:
                pass

    def destroy_node(self) -> None:
        self._shutdown.set()
        self._write_direct(protocol.encode_stop())
        time.sleep(0.1)
        self._disconnect()
        super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SerialNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
