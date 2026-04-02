"""
Corner detection and handling strategies.

Detection: analyze LIDAR scan to find approaching corner.
Handling: compute steering to execute the turn.
"""

from __future__ import annotations

import logging
from abc import ABC, abstractmethod

from perception.world_state import WorldState

logger = logging.getLogger(__name__)


class CornerStrategy(ABC):
    """Base class for corner detection and handling."""

    @abstractmethod
    def detect(self, scan: dict[int, float]) -> str | None:
        """
        Detect if a corner is ahead.

        Args:
            scan: LIDAR scan dict (angle -> distance mm).

        Returns:
            "LEFT", "RIGHT", or None.
        """
        ...

    @abstractmethod
    def compute(self, direction: str, world: WorldState) -> tuple[int, int]:
        """
        Compute speed and steering to execute corner turn.

        Args:
            direction: "LEFT" or "RIGHT".
            world: Current world state.

        Returns:
            (speed, steering) tuple.
        """
        ...


class LidarCornerDetection(CornerStrategy):
    """
    Detect corners using LIDAR front distance.

    When front distance drops below threshold, a corner is ahead.
    Direction is determined by which side has more open space.
    """

    def __init__(self, threshold: int = 400, slow_speed: int = 35, steering_center: int = 90, turn_offset: int = 25, front_window: int = 5, side_window: int = 15, direction_margin: float = 100):
        self.threshold = threshold
        self.slow_speed = slow_speed
        self.steering_center = steering_center
        self.turn_offset = turn_offset
        self.front_window = front_window
        self.side_window = side_window
        self.direction_margin = direction_margin  # mm — left/right must differ by this much
        self._last_direction: str | None = None  # sticky direction to avoid flipping

    def detect(self, scan: dict[int, float]) -> str | None:
        front = self._average_distance(scan, 0, self.front_window)
        left = self._average_distance(scan, 270, self.side_window)
        right = self._average_distance(scan, 90, self.side_window)

        # Method 1: Front wall close (original)
        front_close = front is not None and front <= self.threshold

        # Method 2: One side opens up (wall disappears or jumps far)
        # This detects corners at longer range by seeing the L-shape
        side_open = False
        open_direction = None
        if left is not None and right is not None:
            ratio = left / right if right > 0 else 999
            if ratio > 2.5:  # Left is much further — opening on left
                side_open = True
                open_direction = "LEFT"
            elif ratio < 0.4:  # Right is much further — opening on right
                side_open = True
                open_direction = "RIGHT"

        # Need front wall visible AND (close enough OR side opening detected)
        if not front_close and not (side_open and front is not None and front <= self.threshold * 1.5):
            return None

        # Determine direction
        if open_direction:
            self._last_direction = open_direction
            return open_direction

        if left is None and right is None:
            return None
        if left is None:
            self._last_direction = "RIGHT"
            return "RIGHT"
        if right is None:
            self._last_direction = "LEFT"
            return "LEFT"

        diff = left - right
        if abs(diff) < self.direction_margin:
            return self._last_direction or "RIGHT"

        result = "LEFT" if diff > 0 else "RIGHT"
        self._last_direction = result
        return result

    def compute(self, direction: str, world: WorldState) -> tuple[int, int]:
        if direction == "LEFT":
            steering = self.steering_center - self.turn_offset
        else:
            steering = self.steering_center + self.turn_offset

        return self.slow_speed, steering

    def _min_distance(self, scan: dict[int, float], center: int, window: int) -> float | None:
        """Get minimum distance in angular window. More reliable for corner detection."""
        distances = []
        for offset in range(-window, window + 1):
            angle = (center + offset) % 360
            if angle in scan:
                distances.append(scan[angle])
        if not distances:
            return None
        return min(distances)

    def _average_distance(self, scan: dict[int, float], center: int, window: int) -> float | None:
        distances = []
        for offset in range(-window, window + 1):
            angle = (center + offset) % 360
            if angle in scan:
                distances.append(scan[angle])
        if not distances:
            return None
        return sum(distances) / len(distances)
