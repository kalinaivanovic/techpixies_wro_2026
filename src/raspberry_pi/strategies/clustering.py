"""
Clustering strategies - Group LIDAR points into objects.

Two approaches:
- OpenCVClustering: Convert to image, dilate, findContours (robust)
- RawScanClustering: Walk angles, group by gap (fast, simple)
"""

from __future__ import annotations

import math
from abc import ABC, abstractmethod
from dataclasses import dataclass
import cv2
import numpy as np


@dataclass
class DetectedObject:
    """One object found by LIDAR clustering."""

    angle: float  # Degrees from forward (0-360)
    distance: float  # mm from robot center
    width: float  # Physical width in mm
    obj_type: str  # "wall", "pillar", "unknown"


class ClusteringStrategy(ABC):
    """Base class for LIDAR clustering algorithms."""

    @abstractmethod
    def find_objects(self, scan: dict[int, float]) -> list[DetectedObject]:
        """
        Group LIDAR scan points into detected objects.

        Args:
            scan: Dict mapping angle (0-359) to distance (mm).

        Returns:
            List of detected objects with position, size, and type.
        """
        ...


class OpenCVClustering(ClusteringStrategy):
    """
    Cluster LIDAR points using OpenCV image processing.

    Algorithm:
    1. Convert scan to bird's eye view image
    2. Dilate to connect nearby points (fills gaps)
    3. Find contours (each contour = one object)
    4. Classify by physical width: small = pillar, large = wall
    """

    def __init__(
        self,
        image_size: int = 500,
        max_range: int = 3000,
        dilate_kernel: int = 7,
        dilate_iterations: int = 2,
        min_area_px: int = 20,
        pillar_max_width: float = 120.0,
    ):
        self.image_size = image_size
        self.max_range = max_range
        self.dilate_kernel = dilate_kernel
        self.dilate_iterations = dilate_iterations
        self.min_area_px = min_area_px
        self.pillar_max_width = pillar_max_width

    def find_objects(self, scan: dict[int, float]) -> list[DetectedObject]:
        if not scan:
            return []

        size = self.image_size
        max_range = self.max_range
        center = size // 2
        scale = (size // 2) / max_range

        # Step 1: Convert polar scan to bird's eye view image
        image = np.zeros((size, size), dtype=np.uint8)

        for angle_deg, distance in scan.items():
            if distance > max_range:
                continue
            angle_rad = math.radians(angle_deg)
            x = int(center + distance * math.sin(angle_rad) * scale)
            y = int(center - distance * math.cos(angle_rad) * scale)
            if 0 <= x < size and 0 <= y < size:
                cv2.circle(image, (x, y), 2, 255, -1)

        # Step 2: Dilate to connect nearby points
        kernel = np.ones(
            (self.dilate_kernel, self.dilate_kernel), np.uint8
        )
        dilated = cv2.dilate(image, kernel, iterations=self.dilate_iterations)

        # Step 3: Find contours
        contours, _ = cv2.findContours(
            dilated, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )

        # Step 4: Convert contours to DetectedObjects
        objects = []
        for contour in contours:
            area = cv2.contourArea(contour)
            if area < self.min_area_px:
                continue

            # Bounding box and centroid
            x, y, w, h = cv2.boundingRect(contour)
            M = cv2.moments(contour)
            if M["m00"] == 0:
                continue
            cx = int(M["m10"] / M["m00"])
            cy = int(M["m01"] / M["m00"])

            # Convert pixel centroid back to polar coordinates
            dx = cx - center
            dy = center - cy  # Y is flipped in image
            distance_mm = math.sqrt(dx * dx + dy * dy) / scale
            angle_deg = math.degrees(math.atan2(dx, dy)) % 360

            # Physical width from bounding box
            width_mm = max(w, h) / scale

            # Classify by size
            if width_mm < self.pillar_max_width:
                obj_type = "pillar"
            else:
                obj_type = "wall"

            objects.append(
                DetectedObject(
                    angle=angle_deg,
                    distance=distance_mm,
                    width=width_mm,
                    obj_type=obj_type,
                )
            )

        return objects


class RawScanClustering(ClusteringStrategy):
    """
    Cluster LIDAR points by walking through angles sequentially.

    Algorithm:
    1. Sort scan points by angle
    2. Group consecutive points if angle gap and distance
       difference are within thresholds
    3. Calculate physical width from angular width + distance
    """

    def __init__(
        self,
        angle_gap: int = 5,
        distance_diff: int = 150,
        min_points: int = 3,
        pillar_max_width: float = 120.0,
    ):
        self.angle_gap = angle_gap
        self.distance_diff = distance_diff
        self.min_points = min_points
        self.pillar_max_width = pillar_max_width

    def find_objects(self, scan: dict[int, float]) -> list[DetectedObject]:
        if not scan:
            return []

        sorted_angles = sorted(scan.keys())
        if not sorted_angles:
            return []

        objects = []
        current_cluster = [sorted_angles[0]]

        for i in range(1, len(sorted_angles)):
            prev = sorted_angles[i - 1]
            curr = sorted_angles[i]

            gap = curr - prev
            dist_diff = abs(scan[curr] - scan[prev])

            if gap <= self.angle_gap and dist_diff < self.distance_diff:
                current_cluster.append(curr)
            else:
                if len(current_cluster) >= self.min_points:
                    objects.append(self._make_object(current_cluster, scan))
                current_cluster = [curr]

        # Don't forget last cluster
        if len(current_cluster) >= self.min_points:
            objects.append(self._make_object(current_cluster, scan))

        return objects

    def _make_object(
        self, angles: list[int], scan: dict[int, float]
    ) -> DetectedObject:
        distances = [scan[a] for a in angles]
        avg_distance = sum(distances) / len(distances)
        center_angle = sum(angles) / len(angles)
        angular_width = max(angles) - min(angles)

        # Convert angular width to physical width at that distance
        width_mm = 2 * avg_distance * math.tan(math.radians(angular_width / 2))

        if width_mm < self.pillar_max_width:
            obj_type = "pillar"
        else:
            obj_type = "wall"

        return DetectedObject(
            angle=center_angle % 360,
            distance=avg_distance,
            width=width_mm,
            obj_type=obj_type,
        )
