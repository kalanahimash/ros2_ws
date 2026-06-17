"""Simple PID controller with anti-windup and output clamping."""

from __future__ import annotations

import time


class PIDController:
    """
    Discrete PID controller.

    Anti-windup: integral is clamped to [-integral_limit, +integral_limit].
    Output is clamped to [-output_limit, +output_limit].
    """

    def __init__(
        self,
        kp: float,
        ki: float,
        kd: float,
        output_limit: float = 90.0,
        integral_limit: float = 30.0,
    ) -> None:
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.output_limit    = output_limit
        self.integral_limit  = integral_limit

        self._integral    = 0.0
        self._last_error  = 0.0
        self._last_time   = time.monotonic()

    def compute(self, error: float) -> float:
        now = time.monotonic()
        dt  = now - self._last_time
        if dt <= 0.0:
            dt = 1e-6
        self._last_time = now

        self._integral += error * dt
        self._integral  = max(-self.integral_limit,
                              min(self.integral_limit, self._integral))

        derivative = (error - self._last_error) / dt
        self._last_error = error

        output = self.kp * error + self.ki * self._integral + self.kd * derivative
        return max(-self.output_limit, min(self.output_limit, output))

    def reset(self) -> None:
        self._integral   = 0.0
        self._last_error = 0.0
        self._last_time  = time.monotonic()
