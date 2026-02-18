"""
Wall detection strategies - Extract wall distances from LIDAR scan.

Simple approach: average distances at fixed angles.
Future: line fitting to distinguish walls from pillars.
"""

from abc import ABC, abstractmethod
from typing import Optional

from perception.world_state import WallInfo


class WallDetectionStrategy(ABC):
    """Base class for wall detection algorithms."""

    @abstractmethod
    def detect_walls(self, scan: dict[int, float]) -> WallInfo:
        """
        Extract wall distances from LIDAR scan.

        Args:
            scan: Dict mapping angle (0-359) to distance (mm).

        Returns:
            WallInfo with left, right, front distances.
        """
        ...


class AverageWallDetection(WallDetectionStrategy):
    """
    Detect walls by averaging LIDAR readings at fixed angles.

    Simple and fast, but can confuse a pillar at 90° with
    the right wall. Good enough for initial testing.
    """

    def __init__(
        self,
        left_angle: int = 270,
        right_angle: int = 90,
        front_angle: int = 0,
        side_window: int = 10,
        front_window: int = 5,
    ):
        self.left_angle = left_angle
        self.right_angle = right_angle
        self.front_angle = front_angle
        self.side_window = side_window
        self.front_window = front_window

    def detect_walls(self, scan: dict[int, float]) -> WallInfo:
        return WallInfo(
            left_distance=self._average_distance(
                scan, self.left_angle, self.side_window
            ),
            right_distance=self._average_distance(
                scan, self.right_angle, self.side_window
            ),
            front_distance=self._average_distance(
                scan, self.front_angle, self.front_window
            ),
        )

    def _average_distance(
        self,
        scan: dict[int, float],
        center: int,
        window: int,
    ) -> Optional[float]:
        """Get average distance around a center angle."""
        distances = []
        for offset in range(-window, window + 1):
            angle = (center + offset) % 360
            if angle in scan:
                distances.append(scan[angle])

        if not distances:
            return None
        return sum(distances) / len(distances)
