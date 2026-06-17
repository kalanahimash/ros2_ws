"""
rover_control — Command arbitration and safety node (QoS fixed).

QoS fix: All subscriptions and publications use consistent profiles.
Rate reduced to 10 Hz to match navigation and reduce CPU on Pi.

Priority (highest → lowest):
  1. Emergency stop (latching — requires explicit reset)
  2. Manual command  (active for manual_timeout s after last received)
  3. Autonomous command

Published topics:
  /rover_cmd  (rover_interfaces/RoverCmd)  RELIABLE

Subscribed topics:
  /manual_cmd     (rover_interfaces/RoverCmd)  RELIABLE
  /auto_cmd       (rover_interfaces/RoverCmd)  RELIABLE
  /emergency_stop (std_msgs/Bool)              RELIABLE / TRANSIENT_LOCAL
"""

from __future__ import annotations

import time

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    DurabilityPolicy,
    HistoryPolicy,
)
from std_msgs.msg import Bool
from rover_interfaces.msg import RoverCmd


_RELIABLE_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
)

_ESTOP_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)


class ControlNode(Node):
    """Merges manual and autonomous drive commands with priority and safety checks."""

    def __init__(self) -> None:
        super().__init__("rover_control")

        self.declare_parameter("manual_timeout",    0.5)
        self.declare_parameter("max_linear_speed",  1.0)
        self.declare_parameter("max_angular_speed", 1.0)
        self.declare_parameter("update_rate",       10.0)

        self._manual_timeout  = self.get_parameter("manual_timeout").value
        self._max_linear      = self.get_parameter("max_linear_speed").value
        self._max_angular     = self.get_parameter("max_angular_speed").value
        update_rate           = self.get_parameter("update_rate").value

        self._estop_active     = False
        self._manual_cmd       = RoverCmd()
        self._auto_cmd         = RoverCmd()
        self._last_manual_time = 0.0

        self._pub_cmd = self.create_publisher(RoverCmd, "/rover_cmd", _RELIABLE_QOS)

        self.create_subscription(RoverCmd, "/manual_cmd",     self._on_manual, _RELIABLE_QOS)
        self.create_subscription(RoverCmd, "/auto_cmd",       self._on_auto,   _RELIABLE_QOS)
        self.create_subscription(Bool,     "/emergency_stop", self._on_estop,  _ESTOP_QOS)

        self._timer = self.create_timer(1.0 / update_rate, self._publish_command)

        self.get_logger().info(
            f"rover_control started (manual_timeout={self._manual_timeout}s, "
            f"rate={update_rate}Hz)"
        )

    def _on_manual(self, msg: RoverCmd) -> None:
        self._manual_cmd       = msg
        self._last_manual_time = time.monotonic()
        if msg.emergency_stop:
            self._estop_active = True
            self.get_logger().warn("Emergency stop via manual_cmd")

    def _on_auto(self, msg: RoverCmd) -> None:
        self._auto_cmd = msg
        if msg.emergency_stop and not self._estop_active:
            self._estop_active = True
            self.get_logger().warn("Emergency stop via auto_cmd")

    def _on_estop(self, msg: Bool) -> None:
        if msg.data:
            self._estop_active = True
            self.get_logger().warn("Emergency stop ACTIVE")
        else:
            self._estop_active = False
            self.get_logger().info("Emergency stop CLEARED")

    def _publish_command(self) -> None:
        out = RoverCmd()

        if self._estop_active:
            out.linear         = 0.0
            out.angular        = 0.0
            out.emergency_stop = True
            self._pub_cmd.publish(out)
            return

        manual_age = time.monotonic() - self._last_manual_time
        use_manual = (
            manual_age < self._manual_timeout
            and (abs(self._manual_cmd.linear) > 0.01
                 or abs(self._manual_cmd.angular) > 0.01)
        )

        src = self._manual_cmd if use_manual else self._auto_cmd

        out.linear         = self._clamp(src.linear,  self._max_linear)
        out.angular        = self._clamp(src.angular, self._max_angular)
        out.emergency_stop = False
        self._pub_cmd.publish(out)

    @staticmethod
    def _clamp(value: float, limit: float) -> float:
        return max(-limit, min(limit, value))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
