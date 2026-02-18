"""
Camera sensor - IMX219-120 with OpenCV color detection.

Detects colored blobs:
- Red (pillars to pass on RIGHT)
- Green (pillars to pass on LEFT)
- Magenta (parking markers)
"""

from __future__ import annotations

import logging
import threading
import time
from dataclasses import dataclass

import cv2
import numpy as np

from config import (
    CAMERA_INDEX,
    CAMERA_WIDTH,
    CAMERA_HEIGHT,
    CAMERA_FOV,
    RED_LOWER1,
    RED_UPPER1,
    RED_LOWER2,
    RED_UPPER2,
    GREEN_LOWER,
    GREEN_UPPER,
    MAGENTA_LOWER,
    MAGENTA_UPPER,
    MIN_CONTOUR_AREA,
)

logger = logging.getLogger(__name__)


@dataclass
class ColorBlob:
    """Detected colored region."""

    color: str  # "red", "green", "magenta"
    angle: float  # Degrees from center (-60 to +60 for 120 FOV)
    x: int  # Pixel x (center)
    y: int  # Pixel y (center)
    width: int  # Pixel width
    height: int  # Pixel height
    area: int  # Pixel area


class Camera:
    """
    IMX219-120 camera with color detection.

    Runs capture in background thread, provides latest detections.

    Usage:
        camera = Camera()
        camera.start()

        blobs = camera.get_blobs()
        for blob in blobs:
            print(f"{blob.color} at {blob.angle}°")

        camera.stop()
    """

    def __init__(
        self,
        index: int = CAMERA_INDEX,
        width: int = CAMERA_WIDTH,
        height: int = CAMERA_HEIGHT,
        fov: float = CAMERA_FOV,
        min_area: int = MIN_CONTOUR_AREA,
        params=None,
    ):
        self.index = index
        self.width = width
        self.height = height
        self.fov = fov
        self.min_area = min_area
        self._params = params  # Optional Parameters instance for live tuning

        self._cap: cv2.VideoCapture | None = None
        self._running = False
        self._thread: threading.Thread | None = None
        self._lock = threading.Lock()

        # Latest detection results
        self._blobs: list[ColorBlob] = []
        self._frame: np.ndarray | None = None
        self._timestamp: float = 0.0

        # Pre-compute color ranges as numpy arrays (fallback when no params)
        self._red_lower1 = np.array(RED_LOWER1)
        self._red_upper1 = np.array(RED_UPPER1)
        self._red_lower2 = np.array(RED_LOWER2)
        self._red_upper2 = np.array(RED_UPPER2)
        self._green_lower = np.array(GREEN_LOWER)
        self._green_upper = np.array(GREEN_UPPER)
        self._magenta_lower = np.array(MAGENTA_LOWER)
        self._magenta_upper = np.array(MAGENTA_UPPER)

    @property
    def is_running(self) -> bool:
        return self._running

    def start(self) -> bool:
        """Start camera capture in background thread."""
        if self._running:
            logger.warning("Camera already running")
            return True

        try:
            self._cap = cv2.VideoCapture(self.index)
            self._cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
            self._cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)

            if not self._cap.isOpened():
                logger.error("Failed to open camera")
                return False

            self._running = True
            self._thread = threading.Thread(target=self._capture_loop, daemon=True)
            self._thread.start()

            logger.info(f"Camera started: {self.width}x{self.height}")
            return True

        except Exception as e:
            logger.error(f"Failed to start camera: {e}")
            self._running = False
            return False

    def stop(self):
        """Stop camera capture."""
        self._running = False

        if self._thread:
            self._thread.join(timeout=2.0)
            self._thread = None

        if self._cap:
            self._cap.release()
            self._cap = None

        logger.info("Camera stopped")

    def get_blobs(self) -> list[ColorBlob]:
        """Get latest detected color blobs."""
        with self._lock:
            return self._blobs.copy()

    def get_frame(self) -> np.ndarray | None:
        """Get latest camera frame (BGR format)."""
        with self._lock:
            if self._frame is not None:
                return self._frame.copy()
            return None

    def get_timestamp(self) -> float:
        """Get timestamp of latest frame."""
        with self._lock:
            return self._timestamp

    def get_red_blobs(self) -> list[ColorBlob]:
        """Get red blobs (pillars to pass on RIGHT)."""
        return [b for b in self.get_blobs() if b.color == "red"]

    def get_green_blobs(self) -> list[ColorBlob]:
        """Get green blobs (pillars to pass on LEFT)."""
        return [b for b in self.get_blobs() if b.color == "green"]

    def get_magenta_blobs(self) -> list[ColorBlob]:
        """Get magenta blobs (parking markers)."""
        return [b for b in self.get_blobs() if b.color == "magenta"]

    def get_jpeg_frame(self, draw_boxes: bool = True, quality: int = 80) -> bytes | None:
        """Get latest frame as JPEG bytes, optionally with bounding boxes drawn.

        Args:
            draw_boxes: If True, draw bounding boxes around detected blobs.
            quality: JPEG compression quality (0-100).

        Returns:
            JPEG bytes, or None if no frame available.
        """
        with self._lock:
            if self._frame is None:
                return None
            frame = self._frame.copy()
            blobs = self._blobs.copy()

        if draw_boxes:
            colors = {
                "red": (0, 0, 255),
                "green": (0, 255, 0),
                "magenta": (255, 0, 255),
            }
            for blob in blobs:
                bgr = colors.get(blob.color, (255, 255, 255))
                x = blob.x - blob.width // 2
                y = blob.y - blob.height // 2
                cv2.rectangle(frame, (x, y), (x + blob.width, y + blob.height), bgr, 2)
                cv2.putText(
                    frame,
                    f"{blob.color} {blob.angle:.0f}deg",
                    (x, y - 8),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    bgr,
                    1,
                )

        ret, jpeg = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, quality])
        if not ret:
            return None
        return jpeg.tobytes()

    def get_jpeg_mask(self, color: str, quality: int = 80) -> bytes | None:
        """Get color mask as JPEG bytes.

        Args:
            color: "red", "green", or "magenta".
            quality: JPEG compression quality (0-100).

        Returns:
            JPEG bytes of the binary mask, or None if no frame available.
        """
        with self._lock:
            if self._frame is None:
                return None
            frame = self._frame.copy()

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        if color == "red":
            rl1, ru1 = self._range("red1")
            rl2, ru2 = self._range("red2")
            mask1 = cv2.inRange(hsv, rl1, ru1)
            mask2 = cv2.inRange(hsv, rl2, ru2)
            mask = cv2.bitwise_or(mask1, mask2)
        elif color == "green":
            gl, gu = self._range("green")
            mask = cv2.inRange(hsv, gl, gu)
        elif color == "magenta":
            ml, mu = self._range("magenta")
            mask = cv2.inRange(hsv, ml, mu)
        else:
            return None

        ret, jpeg = cv2.imencode(".jpg", mask, [cv2.IMWRITE_JPEG_QUALITY, quality])
        if not ret:
            return None
        return jpeg.tobytes()

    def _capture_loop(self):
        """Background capture and detection thread."""
        while self._running:
            try:
                ret, frame = self._cap.read()
                if not ret:
                    continue

                # Detect colored blobs
                blobs = self._detect_blobs(frame)

                with self._lock:
                    self._blobs = blobs
                    self._frame = frame
                    self._timestamp = time.time()

            except Exception as e:
                if self._running:
                    logger.error(f"Camera capture error: {e}")

    def _range(self, color: str):
        """Get (lower, upper) HSV numpy arrays for a color.

        Reads from live Parameters if available, otherwise uses
        pre-computed arrays from config.py defaults.
        """
        p = self._params
        if p is not None:
            if color == "red1":
                return (np.array([p.red_h_min1, p.red_s_min1, p.red_v_min1]),
                        np.array([p.red_h_max1, p.red_s_max1, p.red_v_max1]))
            if color == "red2":
                return (np.array([p.red_h_min2, p.red_s_min2, p.red_v_min2]),
                        np.array([p.red_h_max2, p.red_s_max2, p.red_v_max2]))
            if color == "green":
                return (np.array([p.green_h_min, p.green_s_min, p.green_v_min]),
                        np.array([p.green_h_max, p.green_s_max, p.green_v_max]))
            if color == "magenta":
                return (np.array([p.magenta_h_min, p.magenta_s_min, p.magenta_v_min]),
                        np.array([p.magenta_h_max, p.magenta_s_max, p.magenta_v_max]))
        # Fallback to pre-computed arrays
        if color == "red1":
            return (self._red_lower1, self._red_upper1)
        if color == "red2":
            return (self._red_lower2, self._red_upper2)
        if color == "green":
            return (self._green_lower, self._green_upper)
        return (self._magenta_lower, self._magenta_upper)

    def _detect_blobs(self, frame: np.ndarray) -> list[ColorBlob]:
        """Detect colored blobs in frame."""
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        blobs = []

        min_area = self._params.min_contour_area if self._params else self.min_area

        # Detect red (two ranges because red wraps around hue)
        rl1, ru1 = self._range("red1")
        rl2, ru2 = self._range("red2")
        mask_red1 = cv2.inRange(hsv, rl1, ru1)
        mask_red2 = cv2.inRange(hsv, rl2, ru2)
        mask_red = cv2.bitwise_or(mask_red1, mask_red2)
        blobs.extend(self._find_blobs(mask_red, "red", min_area))

        # Detect green
        gl, gu = self._range("green")
        mask_green = cv2.inRange(hsv, gl, gu)
        blobs.extend(self._find_blobs(mask_green, "green", min_area))

        # Detect magenta
        ml, mu = self._range("magenta")
        mask_magenta = cv2.inRange(hsv, ml, mu)
        blobs.extend(self._find_blobs(mask_magenta, "magenta", min_area))

        return blobs

    def _find_blobs(self, mask: np.ndarray, color: str, min_area: int | None = None) -> list[ColorBlob]:
        """Find blobs in a binary mask."""
        if min_area is None:
            min_area = self.min_area
        blobs = []

        # Clean up mask
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.erode(mask, kernel, iterations=1)
        mask = cv2.dilate(mask, kernel, iterations=2)

        # Find contours
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        for contour in contours:
            area = cv2.contourArea(contour)
            if area < min_area:
                continue

            x, y, w, h = cv2.boundingRect(contour)
            center_x = x + w // 2
            center_y = y + h // 2

            # Convert pixel X to angle from center
            angle = self._pixel_to_angle(center_x)

            blobs.append(
                ColorBlob(
                    color=color,
                    angle=angle,
                    x=center_x,
                    y=center_y,
                    width=w,
                    height=h,
                    area=int(area),
                )
            )

        return blobs

    def _pixel_to_angle(self, pixel_x: int) -> float:
        """Convert X pixel position to angle from center."""
        normalized = (pixel_x - self.width / 2) / (self.width / 2)
        return normalized * (self.fov / 2)

    def __enter__(self):
        """Context manager entry."""
        self.start()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        """Context manager exit."""
        self.stop()
