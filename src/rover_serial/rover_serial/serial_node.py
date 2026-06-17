"""
rover_serial — Serial communication node.

Bridges ROS2 topics to the Arduino over USB serial using the
COMMAND:VALUE protocol defined in docs/08_Serial_Protocol.md.

Published topics:
  /distance       (std_msgs/Float32)
  /system_status  (rover_interfaces/SystemStatus)
  /pan_angle      (std_msgs/Float32)
  /tilt_angle     (std_msgs/Float32)

Subscribed topics:
  /rover_cmd  (rover_interfaces/RoverCmd)
  /pan_cmd    (std_msgs/Float32)
  /tilt_cmd   (std_msgs/Float32)
"""

from __future__ import annotations

import queue
import threading
import time
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from std_msgs.msg import Float32, Bool
from rover_interfaces.msg import RoverCmd, SystemStatus

try:
    import serial
    import serial.tools.list_ports
    SERIAL_AVAILABLE = True
except ImportError:
    SERIAL_AVAILABLE = False

from rover_serial import protocol


# QoS for reliable command topics
_RELIABLE_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=5,
)

# QoS for fast sensor data
_SENSOR_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)


class SerialNode(Node):
    """ROS2 node that manages bidirectional USB serial communication with Arduino."""

    def __init__(self) -> None:
        super().__init__("rover_serial")

        # Parameters
        self.declare_parameter("serial_port", "/dev/ttyUSB0")
        self.declare_parameter("baud_rate", 115200)
        self.declare_parameter("reconnect_interval", 3.0)
        self.declare_parameter("heartbeat_interval", 1.0)
        self.declare_parameter("timeout", 2.0)

        self._port       = self.get_parameter("serial_port").value
        self._baud       = self.get_parameter("baud_rate").value
        self._reconnect  = self.get_parameter("reconnect_interval").value
        self._hb_interval = self.get_parameter("heartbeat_interval").value
        self._timeout    = self.get_parameter("timeout").value

        # Serial state
        self._ser: Optional[serial.Serial] = None
        self._serial_lock = threading.Lock()
        self._send_queue: queue.Queue[bytes] = queue.Queue(maxsize=50)
        self._connected = False

        # Publishers
        self._pub_distance = self.create_publisher(Float32, "/distance", _SENSOR_QOS)
        self._pub_status   = self.create_publisher(SystemStatus, "/system_status", _RELIABLE_QOS)
        self._pub_pan      = self.create_publisher(Float32, "/pan_angle", _SENSOR_QOS)
        self._pub_tilt     = self.create_publisher(Float32, "/tilt_angle", _SENSOR_QOS)

        # Subscribers
        self.create_subscription(RoverCmd,  "/rover_cmd", self._on_rover_cmd, _RELIABLE_QOS)
        self.create_subscription(Float32,   "/pan_cmd",   self._on_pan_cmd,   _RELIABLE_QOS)
        self.create_subscription(Float32,   "/tilt_cmd",  self._on_tilt_cmd,  _RELIABLE_QOS)

        # Heartbeat timer
        self._hb_timer = self.create_timer(self._hb_interval, self._send_heartbeat)

        # Background serial reader thread
        self._reader_thread = threading.Thread(target=self._reader_loop, daemon=True)
        self._reader_thread.start()

        # Background serial writer thread
        self._writer_thread = threading.Thread(target=self._writer_loop, daemon=True)
        self._writer_thread.start()

        # Connection management timer
        self._conn_timer = self.create_timer(self._reconnect, self._ensure_connected)
        self._ensure_connected()

        self.get_logger().info(f"rover_serial started. Port={self._port} Baud={self._baud}")

    # ─── Connection Management ──────────────────────────────────────────────

    def _ensure_connected(self) -> None:
        if self._connected:
            return
        if not SERIAL_AVAILABLE:
            self.get_logger().warn("pyserial not installed — running in simulation mode")
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
                )
            self._connected = True
            self.get_logger().info(f"Serial connected: {self._port}")
            # Send reset + heartbeat on reconnect
            self._enqueue(protocol.encode_reset())
            self._enqueue(protocol.encode_heartbeat())
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
        self.get_logger().warn("Serial disconnected — will retry")

    # ─── Background Threads ─────────────────────────────────────────────────

    def _reader_loop(self) -> None:
        """Continuously read lines from serial and post to ROS2 from this thread."""
        while rclpy.ok():
            if not self._connected or self._ser is None:
                time.sleep(0.1)
                continue
            try:
                with self._serial_lock:
                    line = self._ser.readline().decode("ascii", errors="replace")
                if line:
                    self._dispatch_inbound(line)
            except Exception as exc:
                self.get_logger().warn(f"Serial read error: {exc}")
                self._disconnect()
                time.sleep(self._reconnect)

    def _writer_loop(self) -> None:
        """Drain the send queue and write packets to serial."""
        while rclpy.ok():
            try:
                pkt = self._send_queue.get(timeout=0.1)
                if not self._connected or self._ser is None:
                    continue
                try:
                    with self._serial_lock:
                        self._ser.write(pkt)
                        self._ser.flush()
                except Exception as exc:
                    self.get_logger().warn(f"Serial write error: {exc}")
                    self._disconnect()
            except queue.Empty:
                pass

    # ─── Inbound Packet Dispatch ────────────────────────────────────────────

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

        elif cmd == "HB":
            pass  # heartbeat echo — no action needed

        elif cmd == "DBG":
            self.get_logger().debug(f"Arduino: {val}")

    # ─── ROS2 Subscribers ───────────────────────────────────────────────────

    def _on_rover_cmd(self, msg: RoverCmd) -> None:
        if msg.emergency_stop:
            self._enqueue(protocol.encode_estop())
            return
        # Convert normalised [-1,1] to raw [-255,255]
        left  = int(msg.linear * 255 - msg.angular * 128)
        right = int(msg.linear * 255 + msg.angular * 128)
        left  = max(-255, min(255, left))
        right = max(-255, min(255, right))
        self._enqueue(protocol.encode_motor(left, right))

    def _on_pan_cmd(self, msg: Float32) -> None:
        self._enqueue(protocol.encode_pan(msg.data))

    def _on_tilt_cmd(self, msg: Float32) -> None:
        self._enqueue(protocol.encode_tilt(msg.data))

    # ─── Timers ─────────────────────────────────────────────────────────────

    def _send_heartbeat(self) -> None:
        self._enqueue(protocol.encode_heartbeat())

    # ─── Helpers ────────────────────────────────────────────────────────────

    def _enqueue(self, pkt: bytes) -> None:
        try:
            self._send_queue.put_nowait(pkt)
        except queue.Full:
            self.get_logger().warn("Send queue full — dropping packet")

    def destroy_node(self) -> None:
        self._enqueue(protocol.encode_stop())
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
