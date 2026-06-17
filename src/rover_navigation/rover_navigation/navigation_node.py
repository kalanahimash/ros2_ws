"""
rover_navigation — Finite state machine obstacle avoidance.

States: IDLE → FORWARD → STOP → REVERSE → TURN_LEFT|TURN_RIGHT → FORWARD
        Any → EMERGENCY_STOP (on /emergency_stop or distance < emergency_threshold)

Published topics:
  /auto_cmd           (rover_interfaces/RoverCmd)
  /navigation_status  (rover_interfaces/NavigationStatus)

Subscribed topics:
  /distance          (std_msgs/Float32)
  /autonomous_enable (std_msgs/Bool)
  /emergency_stop    (std_msgs/Bool)
"""

from __future__ import annotations

import random
import time
from enum import Enum, auto

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from std_msgs.msg import Float32, Bool
from rover_interfaces.msg import RoverCmd, NavigationStatus

from rover_navigation.distance_filter import DistanceFilter


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


class NavState(Enum):
    IDLE            = auto()
    FORWARD         = auto()
    STOP            = auto()
    REVERSE         = auto()
    TURN_LEFT       = auto()
    TURN_RIGHT      = auto()
    EMERGENCY_STOP  = auto()


class NavigationNode(Node):
    """Obstacle avoidance FSM. Publishes drive commands when autonomous mode is active."""

    def __init__(self) -> None:
        super().__init__("rover_navigation")

        # ── Parameters ──────────────────────────────────────────────────────
        self.declare_parameter("stop_threshold",      30.0)
        self.declare_parameter("emergency_threshold", 10.0)
        self.declare_parameter("forward_speed",       0.6)
        self.declare_parameter("reverse_speed",       0.5)
        self.declare_parameter("turn_speed",          0.5)
        self.declare_parameter("stop_duration",       0.5)
        self.declare_parameter("reverse_duration",    0.8)
        self.declare_parameter("turn_duration",       0.6)
        self.declare_parameter("filter_window",       5)
        self.declare_parameter("distance_timeout",    3.0)
        self.declare_parameter("update_rate",         20.0)

        self._stop_thresh   = self.get_parameter("stop_threshold").value
        self._estop_thresh  = self.get_parameter("emergency_threshold").value
        self._fwd_speed     = self.get_parameter("forward_speed").value
        self._rev_speed     = self.get_parameter("reverse_speed").value
        self._turn_speed    = self.get_parameter("turn_speed").value
        self._stop_dur      = self.get_parameter("stop_duration").value
        self._rev_dur       = self.get_parameter("reverse_duration").value
        self._turn_dur      = self.get_parameter("turn_duration").value
        self._dist_timeout  = self.get_parameter("distance_timeout").value
        update_rate         = self.get_parameter("update_rate").value

        self._filter = DistanceFilter(
            window_size=self.get_parameter("filter_window").value
        )

        # ── State ───────────────────────────────────────────────────────────
        self._state              = NavState.IDLE
        self._state_entered      = time.monotonic()
        self._autonomous_enabled = False
        self._estop_active       = False
        self._distance           = 400.0
        self._last_distance_time = time.monotonic()

        # ── Publishers ──────────────────────────────────────────────────────
        self._pub_cmd    = self.create_publisher(RoverCmd,          "/auto_cmd",          _RELIABLE_QOS)
        self._pub_status = self.create_publisher(NavigationStatus,  "/navigation_status", _RELIABLE_QOS)

        # ── Subscribers ─────────────────────────────────────────────────────
        self.create_subscription(Float32, "/distance",          self._on_distance,    _RELIABLE_QOS)
        self.create_subscription(Bool,    "/autonomous_enable", self._on_auto_enable, _RELIABLE_QOS)
        self.create_subscription(Bool,    "/emergency_stop",    self._on_estop,       _ESTOP_QOS)

        # ── FSM update timer ────────────────────────────────────────────────
        self._timer = self.create_timer(1.0 / update_rate, self._update_fsm)

        self.get_logger().info("rover_navigation started (state=IDLE)")

    # ─── Subscriber Callbacks ───────────────────────────────────────────────

    def _on_distance(self, msg: Float32) -> None:
        self._distance = self._filter.update(msg.data)
        self._last_distance_time = time.monotonic()

        # Immediate emergency stop on critical proximity
        if self._distance < self._estop_thresh and self._autonomous_enabled:
            if self._state != NavState.EMERGENCY_STOP:
                self.get_logger().warn(
                    f"CRITICAL DISTANCE {self._distance:.1f} cm — emergency stop"
                )
                self._transition(NavState.EMERGENCY_STOP)

    def _on_auto_enable(self, msg: Bool) -> None:
        self._autonomous_enabled = msg.data
        if msg.data:
            if self._state == NavState.IDLE:
                self._transition(NavState.FORWARD)
            self.get_logger().info("Autonomous navigation ENABLED")
        else:
            self._transition(NavState.IDLE)
            self.get_logger().info("Autonomous navigation DISABLED")

    def _on_estop(self, msg: Bool) -> None:
        self._estop_active = msg.data
        if msg.data:
            self._transition(NavState.EMERGENCY_STOP)
        elif self._state == NavState.EMERGENCY_STOP:
            self._transition(NavState.IDLE)

    # ─── FSM ────────────────────────────────────────────────────────────────

    def _update_fsm(self) -> None:
        if not self._autonomous_enabled or self._estop_active:
            return

        # Distance sensor timeout safety
        if time.monotonic() - self._last_distance_time > self._dist_timeout:
            if self._state not in (NavState.IDLE, NavState.EMERGENCY_STOP):
                self.get_logger().warn("Distance timeout — stopping")
                self._transition(NavState.IDLE)
                return

        age = time.monotonic() - self._state_entered

        if self._state == NavState.FORWARD:
            if self._distance < self._stop_thresh:
                self._transition(NavState.STOP)
            else:
                self._publish_cmd(self._fwd_speed, 0.0)

        elif self._state == NavState.STOP:
            self._publish_cmd(0.0, 0.0)
            if age >= self._stop_dur:
                self._transition(NavState.REVERSE)

        elif self._state == NavState.REVERSE:
            self._publish_cmd(-self._rev_speed, 0.0)
            if age >= self._rev_dur:
                turn = NavState.TURN_LEFT if random.random() < 0.5 else NavState.TURN_RIGHT
                self._transition(turn)

        elif self._state == NavState.TURN_LEFT:
            self._publish_cmd(0.0, -self._turn_speed)
            if age >= self._turn_dur:
                self._transition(NavState.FORWARD)

        elif self._state == NavState.TURN_RIGHT:
            self._publish_cmd(0.0, self._turn_speed)
            if age >= self._turn_dur:
                self._transition(NavState.FORWARD)

        elif self._state == NavState.EMERGENCY_STOP:
            self._publish_cmd(0.0, 0.0, emergency_stop=True)

        self._publish_status()

    # ─── Helpers ─────────────────────────────────────────────────────────────

    def _transition(self, new_state: NavState) -> None:
        self.get_logger().info(f"NAV: {self._state.name} → {new_state.name}")
        self._state         = new_state
        self._state_entered = time.monotonic()

    def _publish_cmd(
        self, linear: float, angular: float, emergency_stop: bool = False
    ) -> None:
        msg = RoverCmd()
        msg.linear        = float(linear)
        msg.angular       = float(angular)
        msg.emergency_stop = emergency_stop
        self._pub_cmd.publish(msg)

    def _publish_status(self) -> None:
        msg = NavigationStatus()
        msg.state              = self._state.name
        msg.distance_ahead     = float(self._distance)
        msg.obstacle_detected  = self._distance < self._stop_thresh
        msg.state_age_ms       = int((time.monotonic() - self._state_entered) * 1000)
        msg.autonomous_enabled = self._autonomous_enabled
        self._pub_status.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = NavigationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
