"""
Sensor fusion - Combines LIDAR and camera data.

The fusion process:
1. Get wall distances from LIDAR
2. Detect corner via CornerStrategy
3. Find objects via ClusteringStrategy
4. Match camera color blobs with LIDAR objects
5. Produce WorldState with confirmed pillars
"""

import logging
import time
from typing import Optional

from config import ANGLE_MATCH_THRESHOLD, PILLAR_SIZE_MIN, PILLAR_SIZE_MAX
from sensors import Lidar, Camera
from sensors.camera import ColorBlob
from strategies import (
    ClusteringStrategy,
    CornerStrategy,
    DetectedObject,
    OpenCVClustering,
    LidarCornerDetection,
    WallDetectionStrategy,
    AverageWallDetection,
)
from .world_state import WorldState, Pillar

logger = logging.getLogger(__name__)


class SensorFusion:
    """
    Fuses LIDAR and camera data into WorldState.

    Usage:
        fusion = SensorFusion(lidar, camera, get_encoder)

        # In control loop:
        world_state = fusion.update()

        # With custom strategies:
        fusion = SensorFusion(
            lidar, camera, get_encoder,
            clustering=RawScanClustering(),
            corner=LidarCornerDetection(threshold=500),
        )
    """

    def __init__(
        self,
        lidar: Lidar,
        camera: Camera,
        get_encoder: callable,
        clustering: ClusteringStrategy = None,
        corner: CornerStrategy = None,
        wall_detection: WallDetectionStrategy = None,
    ):
        self.lidar = lidar
        self.camera = camera
        self.get_encoder = get_encoder
        self.clustering = clustering or OpenCVClustering()
        self.corner = corner or LidarCornerDetection()
        self.wall_detection = wall_detection or AverageWallDetection()

    def update(self) -> WorldState:
        """
        Fuse current sensor data into WorldState.

        Returns:
            WorldState with current perception
        """
        # Get sensor data
        scan = self.lidar.get_scan()
        blobs = self.camera.get_blobs()
        encoder = self.get_encoder()

        # Extract wall distances (strategy)
        walls = self.wall_detection.detect_walls(scan)

        # Detect corner (strategy)
        corner_ahead = self.corner.detect(scan)

        # Find LIDAR objects (strategy)
        objects = self.clustering.find_objects(scan)

        # Match camera blobs with LIDAR objects
        pillars = self._match_pillars(objects, blobs)

        # Detect parking marker
        parking_marker = self._detect_parking(blobs, scan)

        return WorldState(
            timestamp=time.time(),
            encoder_pos=encoder,
            walls=walls,
            pillars=pillars,
            corner_ahead=corner_ahead,
            parking_marker=parking_marker,
        )

    def _average_distance(
        self,
        scan: dict[int, float],
        center: int,
        window: int = 5,
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

    def _match_pillars(
        self,
        objects: list[DetectedObject],
        blobs: list[ColorBlob],
    ) -> list[Pillar]:
        """Match camera color detections with LIDAR objects."""
        pillars = []
        used_objects = set()

        # Only consider red and green blobs
        pillar_blobs = [b for b in blobs if b.color in ("red", "green")]

        for blob in pillar_blobs:
            best_match = None
            best_angle_diff = float("inf")

            for i, obj in enumerate(objects):
                if i in used_objects:
                    continue

                # Only match pillar-sized objects
                if not (PILLAR_SIZE_MIN <= obj.width <= PILLAR_SIZE_MAX):
                    continue

                # Convert LIDAR angle to camera reference
                # LIDAR: 0=forward, 90=right, 270=left
                # Camera: 0=center, positive=right, negative=left
                lidar_angle = obj.angle
                if lidar_angle > 180:
                    camera_angle = lidar_angle - 360
                else:
                    camera_angle = lidar_angle

                angle_diff = abs(blob.angle - camera_angle)

                if angle_diff < ANGLE_MATCH_THRESHOLD and angle_diff < best_angle_diff:
                    best_match = (i, obj)
                    best_angle_diff = angle_diff

            if best_match is not None:
                idx, obj = best_match
                used_objects.add(idx)
                pillars.append(
                    Pillar(
                        color=blob.color,
                        angle=blob.angle,
                        distance=obj.distance,
                    )
                )

        return pillars

    def _detect_parking(
        self,
        blobs: list[ColorBlob],
        scan: dict[int, float],
    ) -> Optional[float]:
        """Detect parking marker (magenta color)."""
        magenta = [b for b in blobs if b.color == "magenta"]
        if not magenta:
            return None

        # Get largest magenta blob
        largest = max(magenta, key=lambda b: b.area)

        # Try to get distance from LIDAR
        lidar_angle = int(largest.angle) % 360
        if largest.angle < 0:
            lidar_angle = int(360 + largest.angle) % 360

        distance = self._average_distance(scan, lidar_angle, window=5)
        return distance if distance else 1000.0  # Default if can't measure
