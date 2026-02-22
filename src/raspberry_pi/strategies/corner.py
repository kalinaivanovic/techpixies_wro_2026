"""
Corner detection and handling strategies.

Detection: analyze LIDAR scan to find approaching corner.
Handling: compute steering to execute the turn.
"""

from __future__ import annotations

from abc import ABC, abstractmethod

from perception.world_state import WorldState


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

    def __init__(
        self,
        threshold: int = 400,
        slow_speed: int = 35,
        steering_center: int = 90,
        turn_offset: int = 25,
        front_window: int = 5,
        side_window: int = 15,
    ):
        self.threshold = threshold
        self.slow_speed = slow_speed
        self.steering_center = steering_center
        self.turn_offset = turn_offset
        self.front_window = front_window
        self.side_window = side_window

    def detect(self, scan: dict[int, float]) -> str | None:
        front = self._average_distance(scan, 0, self.front_window)

        if front is None or front > self.threshold:
            return None

        # Corner detected — which direction?
        left = self._average_distance(scan, 270, self.side_window)
        right = self._average_distance(scan, 90, self.side_window)

        if left is None and right is None:
            return None
        if left is None:
            return "RIGHT"
        if right is None:
            return "LEFT"

        return "LEFT" if left > right else "RIGHT"

    def compute(self, direction: str, world: WorldState) -> tuple[int, int]:
        if direction == "LEFT":
            steering = self.steering_center - self.turn_offset
        else:
            steering = self.steering_center + self.turn_offset

        return self.slow_speed, steering

    def _average_distance(
        self,
        scan: dict[int, float],
        center: int,
        window: int,
    ) -> float | None:
        distances = []
        for offset in range(-window, window + 1):
            angle = (center + offset) % 360
            if angle in scan:
                distances.append(scan[angle])
        if not distances:
            return None
        return sum(distances) / len(distances)
