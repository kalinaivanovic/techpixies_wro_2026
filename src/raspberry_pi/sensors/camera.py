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
    CAMERA_FOV,
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
        params = Parameters.load()
        camera = Camera(params=params)
        camera.start()

        blobs = camera.get_blobs()
        for blob in blobs:
            print(f"{blob.color} at {blob.angle}°")

        camera.stop()
    """

    # IMX219 full-sensor binned mode (2x2 binning, 4:3, full FOV)
    # picamera2 ISP downscales from this to the requested output size
    SENSOR_FULL_WIDTH = 1640
    SENSOR_FULL_HEIGHT = 1232

    def __init__(self, params, index: int = CAMERA_INDEX, fov: float = CAMERA_FOV):
        self.params = params
        self.index = index
        self.fov = fov
        # Actual capture dimensions (set on start from params)
        self.width = params.camera_width
        self.height = params.camera_height

        self._cap: cv2.VideoCapture | None = None
        self._picam = None
        self._use_picamera = False
        self._running = False
        self._thread: threading.Thread | None = None
        self._lock = threading.Lock()

        # Latest detection results
        self._blobs: list[ColorBlob] = []
        self._frame: np.ndarray | None = None
        self._timestamp: float = 0.0

    @property
    def is_running(self) -> bool:
        return self._running

    def start(self) -> bool:
        """Start camera capture in background thread.

        Tries picamera2 (CSI camera) first, falls back to OpenCV VideoCapture.
        """
        if self._running:
            logger.warning("Camera already running")
            return True

        # Read resolution from params (may have changed since __init__)
        self.width = self.params.camera_width
        self.height = self.params.camera_height

        try:
            # Try picamera2 first (Raspberry Pi CSI camera)
            from picamera2 import Picamera2

            self._picam = Picamera2()
            self._picam.configure(self._picam.create_preview_configuration(
                main={"format": "RGB888", "size": (self.width, self.height)},
                # Force full-sensor capture (2x2 binned) to preserve full FOV.
                # Without this, picamera2 may pick a center-crop sensor mode.
                sensor={"output_size": (self.SENSOR_FULL_WIDTH, self.SENSOR_FULL_HEIGHT)},
            ))
            self._picam.start()
            self._use_picamera = True
            logger.info(
                f"Camera started (picamera2 CSI): {self.width}x{self.height} "
                f"from sensor {self.SENSOR_FULL_WIDTH}x{self.SENSOR_FULL_HEIGHT}"
            )

        except (ImportError, Exception) as e:
            # Fall back to OpenCV VideoCapture (USB webcam)
            logger.info(f"picamera2 not available ({e}), trying OpenCV VideoCapture")
            self._picam = None
            self._use_picamera = False

            self._cap = cv2.VideoCapture(self.index)
            self._cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
            self._cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)

            if not self._cap.isOpened():
                logger.error("Failed to open camera")
                return False

            logger.info(f"Camera started (OpenCV): {self.width}x{self.height}")

        self._running = True
        self._thread = threading.Thread(target=self._capture_loop, daemon=True)
        self._thread.start()
        return True

    def stop(self):
        """Stop camera capture."""
        self._running = False

        if self._thread:
            self._thread.join(timeout=2.0)
            self._thread = None

        if hasattr(self, '_picam') and self._picam:
            self._picam.stop()
            self._picam = None

        if self._cap:
            self._cap.release()
            self._cap = None

        logger.info("Camera stopped")

    def restart(self):
        """Restart camera with current params (e.g., after resolution change)."""
        self.stop()
        self.start()

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
                if self._use_picamera:
                    # picamera2 with "RGB888" format returns BGR in memory
                    # (libcamera naming convention), so no conversion needed
                    frame = self._picam.capture_array()
                else:
                    ret, frame = self._cap.read()
                    if not ret:
                        continue

                # Detect colored blobs
                blobs = self._detect_blobs(frame)

                with self._lock:
                    self._blobs = blobs
                    self._frame = frame
                    self._timestamp = time.time()

                # Yield CPU to other threads (keepalive, web server)
                time.sleep(0.005)

            except Exception as e:
                if self._running:
                    logger.error(f"Camera capture error: {e}")

    def _range(self, color: str):
        """Get (lower, upper) HSV numpy arrays for a color from Parameters."""
        p = self.params
        if color == "red1":
            return (np.array([p.red_h_min1, p.red_s_min1, p.red_v_min1]),
                    np.array([p.red_h_max1, p.red_s_max1, p.red_v_max1]))
        if color == "red2":
            return (np.array([p.red_h_min2, p.red_s_min2, p.red_v_min2]),
                    np.array([p.red_h_max2, p.red_s_max2, p.red_v_max2]))
        if color == "green":
            return (np.array([p.green_h_min, p.green_s_min, p.green_v_min]),
                    np.array([p.green_h_max, p.green_s_max, p.green_v_max]))
        return (np.array([p.magenta_h_min, p.magenta_s_min, p.magenta_v_min]),
                np.array([p.magenta_h_max, p.magenta_s_max, p.magenta_v_max]))

    def _detect_blobs(self, frame: np.ndarray) -> list[ColorBlob]:
        """Detect colored blobs in frame."""
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        blobs = []

        min_area = self.params.min_contour_area

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

    def _find_blobs(self, mask: np.ndarray, color: str, min_area: int) -> list[ColorBlob]:
        """Find blobs in a binary mask."""
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
