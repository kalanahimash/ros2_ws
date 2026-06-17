"""Thread-safe shared state for the dashboard."""

from __future__ import annotations

import threading
import time
from dataclasses import dataclass, field
from typing import Optional


@dataclass
class RoverState:
    distance:          float = 400.0
    pan_angle:         float = 90.0
    tilt_angle:        float = 90.0
    battery_voltage:   float = 0.0
    mode:              str   = "IDLE"
    motors_enabled:    bool  = False
    watchdog_ok:       bool  = True
    tracking:          bool  = False
    tracking_state:    str   = "DISABLED"
    tracking_fps:      float = 0.0
    autonomous:        bool  = False
    nav_state:         str   = "IDLE"
    obstacle_detected: bool  = False
    camera_fps:        float = 0.0
    estop_active:      bool  = False
    serial_connected:  bool  = False
    last_update:       float = field(default_factory=time.monotonic)

    # Latest JPEG bytes for MJPEG stream
    latest_jpeg:       Optional[bytes] = None
    jpeg_lock:         threading.Lock = field(default_factory=threading.Lock)

    # General RW lock for all other fields
    lock:              threading.Lock = field(default_factory=threading.Lock)

    def to_dict(self) -> dict:
        with self.lock:
            return {
                "distance":          self.distance,
                "pan_angle":         self.pan_angle,
                "tilt_angle":        self.tilt_angle,
                "battery_voltage":   self.battery_voltage,
                "mode":              self.mode,
                "motors_enabled":    self.motors_enabled,
                "watchdog_ok":       self.watchdog_ok,
                "tracking":          self.tracking,
                "tracking_state":    self.tracking_state,
                "tracking_fps":      self.tracking_fps,
                "autonomous":        self.autonomous,
                "nav_state":         self.nav_state,
                "obstacle_detected": self.obstacle_detected,
                "camera_fps":        self.camera_fps,
                "estop_active":      self.estop_active,
                "serial_connected":  self.serial_connected,
            }
