"""
HSV colour detector.

Detects the largest blob of a configurable colour in a BGR image.
Returns the centroid and area, or None if no target found.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional, Tuple

import numpy as np

try:
    import cv2
    CV2_AVAILABLE = True
except ImportError:
    CV2_AVAILABLE = False


@dataclass
class Detection:
    cx: float          # Centroid X (pixels)
    cy: float          # Centroid Y (pixels)
    area: float        # Contour area (pixels²)
    bbox: Tuple[int, int, int, int]  # (x, y, w, h)


class HSVDetector:
    """Detect a colour target using HSV thresholding and contour analysis."""

    def __init__(
        self,
        hsv_lower: Tuple[int, int, int] = (5, 100, 100),
        hsv_upper: Tuple[int, int, int] = (25, 255, 255),
        min_area:  float = 500.0,
        kernel_size: int = 5,
        erode_iterations:  int = 2,
        dilate_iterations: int = 2,
    ) -> None:
        self.hsv_lower  = np.array(hsv_lower, dtype=np.uint8)
        self.hsv_upper  = np.array(hsv_upper, dtype=np.uint8)
        self.min_area   = min_area
        self._kernel    = (
            cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (kernel_size, kernel_size))
            if CV2_AVAILABLE else None
        )
        self._erode_iter  = erode_iterations
        self._dilate_iter = dilate_iterations

    def detect(self, bgr_frame: np.ndarray) -> Optional[Detection]:
        if not CV2_AVAILABLE:
            return None

        hsv  = cv2.cvtColor(bgr_frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.hsv_lower, self.hsv_upper)

        # Morphological cleanup
        mask = cv2.erode(mask,  self._kernel, iterations=self._erode_iter)
        mask = cv2.dilate(mask, self._kernel, iterations=self._dilate_iter)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return None

        largest = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(largest)
        if area < self.min_area:
            return None

        M = cv2.moments(largest)
        if M["m00"] == 0:
            return None

        cx = M["m10"] / M["m00"]
        cy = M["m01"] / M["m00"]
        x, y, w, h = cv2.boundingRect(largest)

        return Detection(cx=cx, cy=cy, area=area, bbox=(x, y, w, h))

    def draw_overlay(
        self,
        frame: np.ndarray,
        det: Optional[Detection],
        frame_cx: float,
        frame_cy: float,
    ) -> np.ndarray:
        if not CV2_AVAILABLE:
            return frame

        overlay = frame.copy()

        # Draw crosshair at frame centre
        cv2.line(overlay, (int(frame_cx) - 20, int(frame_cy)),
                 (int(frame_cx) + 20, int(frame_cy)), (255, 255, 255), 1)
        cv2.line(overlay, (int(frame_cx), int(frame_cy) - 20),
                 (int(frame_cx), int(frame_cy) + 20), (255, 255, 255), 1)

        if det is not None:
            x, y, w, h = det.bbox
            cv2.rectangle(overlay, (x, y), (x + w, y + h), (0, 255, 0), 2)
            cv2.circle(overlay, (int(det.cx), int(det.cy)), 6, (0, 0, 255), -1)
            cv2.line(overlay, (int(frame_cx), int(frame_cy)),
                     (int(det.cx), int(det.cy)), (0, 200, 255), 1)
            label = f"A={int(det.area)}"
            cv2.putText(overlay, label, (x, y - 8),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 0), 1)

        return overlay
