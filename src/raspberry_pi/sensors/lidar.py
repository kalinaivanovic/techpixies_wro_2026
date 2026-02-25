"""
LIDAR sensor - RPLIDAR C1 driver.

Provides continuous scanning with background thread.
Returns LidarFrame with 360-degree scan data.
"""

from __future__ import annotations

import logging
import math
import threading
import time
from dataclasses import dataclass
import cv2
import numpy as np
from pyrplidar import PyRPlidar

from config import (
    LIDAR_PORT,
    LIDAR_BAUDRATE,
)

logger = logging.getLogger(__name__)


@dataclass
class LidarCluster:
    """Detected cluster from LIDAR bird's eye view."""

    angle: float  # Degrees from forward (0-360)
    distance_mm: float  # Distance from robot center
    width_mm: float  # Estimated physical width
    area_px: int  # Pixel area in image
    bbox_tl: tuple[int, int]  # Bounding box top-left (x, y) in pixels
    bbox_br: tuple[int, int]  # Bounding box bottom-right (x, y) in pixels
    centroid_px: tuple[int, int]  # Centroid (x, y) in pixels


class Lidar:
    """
    RPLIDAR C1 driver with background scanning.

    Usage:
        params = Parameters.load()
        lidar = Lidar(params=params)
        lidar.start()

        # Get latest scan data
        scan = lidar.get_scan()  # dict[angle] -> distance

        lidar.stop()
    """

    def __init__(self, params, port: str = LIDAR_PORT, baudrate: int = LIDAR_BAUDRATE, motor_pwm: int = 660):
        self.params = params
        self.port = port
        self.baudrate = baudrate
        self.motor_pwm = motor_pwm

        self._lidar: PyRPlidar | None = None
        self._running = False
        self._thread: threading.Thread | None = None
        self._lock = threading.Lock()

        # Latest scan data: angle (0-359) -> distance (mm)
        self._scan: dict[int, float] = {}
        self._scan_quality: dict[int, float] = {}  # angle -> avg quality
        self._scan_timestamp: float = 0.0

    @property
    def is_running(self) -> bool:
        return self._running

    def start(self) -> bool:
        """Start LIDAR scanning in background thread."""
        if self._running:
            logger.warning("LIDAR already running")
            return True

        try:
            self._lidar = PyRPlidar()
            self._lidar.connect(port=self.port, baudrate=self.baudrate)
            self._lidar.set_motor_pwm(self.motor_pwm)
            time.sleep(1)  # Let motor spin up

            self._running = True
            self._thread = threading.Thread(target=self._scan_loop, daemon=True)
            self._thread.start()

            logger.info(f"LIDAR started on {self.port}")
            return True

        except Exception as e:
            logger.error(f"Failed to start LIDAR: {e}")
            self._running = False
            return False

    def stop(self):
        """Stop LIDAR scanning."""
        self._running = False

        if self._thread:
            self._thread.join(timeout=2.0)
            self._thread = None

        if self._lidar:
            try:
                self._lidar.stop()
                self._lidar.set_motor_pwm(0)
                self._lidar.disconnect()
            except Exception as e:
                logger.error(f"Error stopping LIDAR: {e}")
            self._lidar = None

        logger.info("LIDAR stopped")

    def get_scan(self) -> dict[int, float]:
        """
        Get latest scan data, filtered by params (max distance, angle, quality).

        Returns:
            Dict mapping angle (0-359) to distance (mm).
            Angle 0 = forward, 90 = right, 180 = back, 270 = left.
        """
        with self._lock:
            scan = self._scan.copy()
        return self._filter_scan(scan)

    def get_timestamp(self) -> float:
        """Get timestamp of latest scan."""
        with self._lock:
            return self._scan_timestamp

    def get_distance(self, angle: int) -> float | None:
        """
        Get distance at specific angle.

        Args:
            angle: Angle in degrees (0-359)

        Returns:
            Distance in mm, or None if no reading
        """
        with self._lock:
            return self._scan.get(angle % 360)

    def get_front(self) -> float | None:
        """Get average distance in front direction (±10 degrees)."""
        return self._get_average_distance(0, window=10)

    def get_left(self) -> float | None:
        """Get average distance to the left (270° ±10)."""
        return self._get_average_distance(270, window=10)

    def get_right(self) -> float | None:
        """Get average distance to the right (90° ±10)."""
        return self._get_average_distance(90, window=10)

    def _get_average_distance(self, center: int, window: int = 10) -> float | None:
        """Get average distance around a center angle."""
        with self._lock:
            distances = []
            for offset in range(-window, window + 1):
                angle = (center + offset) % 360
                if angle in self._scan:
                    distances.append(self._scan[angle])

            if not distances:
                return None
            return sum(distances) / len(distances)

    def _filter_scan(self, scan: dict[int, float]) -> dict[int, float]:
        """Filter scan by angle, max distance, and min quality from params."""
        angle_limit = self.params.lidar_display_angle
        max_dist = self.params.lidar_max_distance
        min_qual = self.params.lidar_min_quality
        quality = self._scan_quality
        filtered = {}
        for angle_deg, distance in scan.items():
            if distance > max_dist:
                continue
            if min_qual > 0 and quality.get(angle_deg, 0) < min_qual:
                continue
            # Convert to signed angle: 0=forward, negative=left, positive=right
            signed = angle_deg if angle_deg <= 180 else angle_deg - 360
            if -angle_limit <= signed <= angle_limit:
                filtered[angle_deg] = distance
        return filtered

    def get_jpeg_frame(self, size: int = 500, quality: int = 80) -> bytes | None:
        """Get bird's eye view with detected clusters as JPEG bytes.

        Uses params.lidar_max_distance and params.lidar_display_angle for filtering.
        """
        scan = self.get_scan()
        if not scan:
            return None
        max_range = self.params.lidar_max_distance

        # Create bird's eye view image
        image = self._scan_to_image(scan, size, max_range)

        # Detect clusters using OpenCV
        clusters = self._find_clusters(image, scan, size, max_range)

        # Draw clusters on image
        display = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
        # Recolor LIDAR points to cyan
        display[image > 0] = (255, 255, 0)  # cyan in BGR

        # Draw robot position
        center = size // 2
        cv2.drawMarker(
            display, (center, center), (0, 200, 0),
            cv2.MARKER_TRIANGLE_UP, 12, 2,
        )

        # Draw range rings
        for r_mm in range(500, max_range + 1, 500):
            r_px = int(r_mm / max_range * (size // 2))
            cv2.circle(display, (center, center), r_px, (40, 40, 40), 1)

        # Draw cluster bounding boxes
        for cluster in clusters:
            # Color by size: small = red (pillar-like), large = green (wall-like)
            if cluster.width_mm < 120:
                color = (0, 0, 255)  # red = pillar candidate
                label = f"{cluster.distance_mm:.0f}mm"
            else:
                color = (0, 180, 0)  # green = wall
                label = f"W {cluster.distance_mm:.0f}mm"

            cv2.rectangle(display, cluster.bbox_tl, cluster.bbox_br, color, 2)
            cv2.putText(
                display, label,
                (cluster.bbox_tl[0], cluster.bbox_tl[1] - 6),
                cv2.FONT_HERSHEY_SIMPLEX, 0.4, color, 1,
            )

        ret, jpeg = cv2.imencode(".jpg", display, [cv2.IMWRITE_JPEG_QUALITY, quality])
        if not ret:
            return None
        return jpeg.tobytes()

    def get_jpeg_raw(self, size: int = 500, quality: int = 80) -> bytes | None:
        """Get raw bird's eye view (points only, no clusters) as JPEG bytes."""
        scan = self.get_scan()
        if not scan:
            return None
        max_range = self.params.lidar_max_distance

        image = self._scan_to_image(scan, size, max_range)
        display = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
        display[image > 0] = (255, 255, 255)

        # Draw robot and range rings
        center = size // 2
        cv2.drawMarker(
            display, (center, center), (0, 200, 0),
            cv2.MARKER_TRIANGLE_UP, 12, 2,
        )
        for r_mm in range(500, max_range + 1, 500):
            r_px = int(r_mm / max_range * (size // 2))
            cv2.circle(display, (center, center), r_px, (40, 40, 40), 1)

        ret, jpeg = cv2.imencode(".jpg", display, [cv2.IMWRITE_JPEG_QUALITY, quality])
        if not ret:
            return None
        return jpeg.tobytes()

    def _scan_to_image(self, scan: dict[int, float], size: int, max_range: int) -> np.ndarray:
        """Convert scan dict to bird's eye view binary image.

        Robot is at center. Forward (angle 0) = up.
        """
        image = np.zeros((size, size), dtype=np.uint8)
        center = size // 2
        scale = (size // 2) / max_range

        for angle_deg, distance in scan.items():
            if distance > max_range:
                continue

            # Convert polar to cartesian
            # Angle 0 = forward (up), 90 = right, clockwise
            angle_rad = math.radians(angle_deg)
            x = int(center + distance * math.sin(angle_rad) * scale)
            y = int(center - distance * math.cos(angle_rad) * scale)

            if 0 <= x < size and 0 <= y < size:
                # Draw a small circle for each point (more visible)
                cv2.circle(image, (x, y), 2, 255, -1)

        return image

    def _find_clusters(self, image: np.ndarray, scan: dict[int, float], size: int, max_range: int) -> list["LidarCluster"]:
        """Find clusters in bird's eye view image using OpenCV."""
        # Dilate to connect nearby points
        kernel = np.ones((7, 7), np.uint8)
        dilated = cv2.dilate(image, kernel, iterations=2)

        # Find contours
        contours, _ = cv2.findContours(
            dilated, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )

        center = size // 2
        scale = (size // 2) / max_range
        clusters = []

        for contour in contours:
            area = cv2.contourArea(contour)
            if area < 20:  # Filter tiny noise
                continue

            x, y, w, h = cv2.boundingRect(contour)

            # Compute centroid
            M = cv2.moments(contour)
            if M["m00"] == 0:
                continue
            cx = int(M["m10"] / M["m00"])
            cy = int(M["m01"] / M["m00"])

            # Convert centroid back to polar (distance + angle from robot)
            dx = cx - center
            dy = center - cy  # Y is flipped
            distance_px = math.sqrt(dx * dx + dy * dy)
            distance_mm = distance_px / scale
            angle_deg = math.degrees(math.atan2(dx, dy)) % 360

            # Estimate physical width
            width_mm = max(w, h) / scale

            clusters.append(
                LidarCluster(
                    angle=angle_deg,
                    distance_mm=distance_mm,
                    width_mm=width_mm,
                    area_px=int(area),
                    bbox_tl=(x, y),
                    bbox_br=(x + w, y + h),
                    centroid_px=(cx, cy),
                )
            )

        return clusters

    def _scan_loop(self):
        """Background scanning thread."""
        try:
            scan_generator = self._lidar.start_scan()
            current_scan: dict[int, list[float]] = {}
            current_quality: dict[int, list[int]] = {}

            for reading in scan_generator():
                if not self._running:
                    break

                angle = int(reading.angle) % 360
                distance = reading.distance
                quality = reading.quality

                # Filter invalid readings
                if distance < self.params.lidar_min_distance:
                    continue
                if quality < self.params.lidar_min_quality:
                    continue

                # Accumulate readings for averaging
                if angle not in current_scan:
                    current_scan[angle] = []
                    current_quality[angle] = []
                current_scan[angle].append(distance)
                current_quality[angle].append(quality)

                # Instant mode: update live scan only for forward-facing angles
                if self.params.lidar_instant:
                    signed = angle if angle <= 180 else angle - 360
                    if -self.params.lidar_display_angle <= signed <= self.params.lidar_display_angle:
                        with self._lock:
                            self._scan[angle] = distance
                            self._scan_quality[angle] = quality
                            self._scan_timestamp = time.time()

                # Full rotation: publish averaged scan
                if angle == 0 and len(current_scan) > 180:
                    averaged = {
                        a: sum(d) / len(d)
                        for a, d in current_scan.items()
                    }
                    avg_quality = {
                        a: sum(q) / len(q)
                        for a, q in current_quality.items()
                    }

                    with self._lock:
                        self._scan = averaged
                        self._scan_quality = avg_quality
                        self._scan_timestamp = time.time()

                    current_scan.clear()
                    current_quality.clear()

        except Exception as e:
            if self._running:
                logger.error(f"LIDAR scan error: {e}")

    def __enter__(self):
        """Context manager entry."""
        self.start()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        """Context manager exit."""
        self.stop()
