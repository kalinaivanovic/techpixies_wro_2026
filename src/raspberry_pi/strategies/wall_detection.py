"""
Wall detection strategies - Extract wall distances from LIDAR scan.

Two approaches:
- AverageWallDetection: average distances at fixed angles (fast, simple)
- ClusteringWallDetection: cluster scan first, use only wall-classified
  objects (avoids pillar contamination)
"""

from __future__ import annotations

from abc import ABC, abstractmethod

from perception.world_state import WallInfo
from .clustering import ClusteringStrategy, DetectedObject, OpenCVClustering


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

    def __init__(self, left_angle: int = 270, right_angle: int = 90, front_angle: int = 0, side_window: int = 10, front_window: int = 5):
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

    def _average_distance(self, scan: dict[int, float], center: int, window: int) -> float | None:
        """Get average distance around a center angle."""
        distances = []
        for offset in range(-window, window + 1):
            angle = (center + offset) % 360
            if angle in scan:
                distances.append(scan[angle])

        if not distances:
            return None
        return sum(distances) / len(distances)


class ClusteringWallDetection(WallDetectionStrategy):
    """
    Detect walls using LIDAR clustering to filter out pillars.

    Algorithm:
    1. Run clustering on the scan (same as used for pillar detection)
    2. Keep only objects classified as "wall" (width >= pillar threshold)
    3. For each cardinal direction (left/right/front), find the closest
       wall object within an angular tolerance

    This avoids the main problem of AverageWallDetection: a pillar
    sitting at 90° won't be mistaken for the right wall, because
    clustering classifies it as "pillar" by its small width.
    """

    def __init__(self, clustering: ClusteringStrategy | None = None, angle_tolerance: float = 45.0):
        self.clustering = clustering or OpenCVClustering()
        self.angle_tolerance = angle_tolerance

    def detect_walls(self, scan: dict[int, float]) -> WallInfo:
        objects = self.clustering.find_objects(scan)

        # Only consider wall-type objects
        walls = [o for o in objects if o.obj_type == "wall"]

        return WallInfo(
            left_distance=self._find_wall_distance(walls, 270),
            right_distance=self._find_wall_distance(walls, 90),
            front_distance=self._find_wall_distance(walls, 0),
        )

    def _find_wall_distance(self, walls: list[DetectedObject], target_angle: float) -> float | None:
        """Find the nearest wall object close to the target angle."""
        best = None
        best_diff = float("inf")

        for wall in walls:
            diff = abs(wall.angle - target_angle)
            if diff > 180:
                diff = 360 - diff

            if diff < self.angle_tolerance and diff < best_diff:
                best = wall
                best_diff = diff

        return best.distance if best else None
