"""Rolling median filter for ultrasonic distance readings."""

from __future__ import annotations

from collections import deque
from typing import Optional


class DistanceFilter:
    """Rolling median filter with configurable window size."""

    def __init__(self, window_size: int = 5, max_value: float = 400.0) -> None:
        self._window     = deque(maxlen=window_size)
        self._max_value  = max_value

    def update(self, value: float) -> float:
        value = max(0.0, min(self._max_value, value))
        self._window.append(value)
        return self.get()

    def get(self) -> float:
        if not self._window:
            return self._max_value
        sorted_vals = sorted(self._window)
        return sorted_vals[len(sorted_vals) // 2]

    def is_ready(self) -> bool:
        return len(self._window) > 0

    def reset(self) -> None:
        self._window.clear()
