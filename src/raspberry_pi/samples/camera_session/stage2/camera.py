"""
Camera - capture frames and detect colored objects.

Stage 3: Full detection with bounding boxes and color masks.

Detects:
- Red blobs (pillars to pass on RIGHT)
- Green blobs (pillars to pass on LEFT)
- Magenta blobs (parking markers)
"""

import threading
import time
from dataclasses import dataclass

import cv2
import numpy as np

# Camera settings
CAMERA_INDEX = 0
CAMERA_WIDTH = 640
CAMERA_HEIGHT = 480
CAMERA_FOV = 120  # degrees (wide-angle lens)

# Minimum blob area in pixels (filter out noise)
MIN_AREA = 300

# HSV color ranges for detection
# (OpenCV uses H: 0-180, S: 0-255, V: 0-255)
#
# Hue scale:
# Red     Orange    Yellow    Green     Cyan      Blue      Magenta   Red
#  0        15        30        60        90       120        150      180
#
# Red wraps around both ends (0-10 AND 160-180), so it needs two ranges.

RED_LOWER1 = np.array([0, 100, 100])
RED_UPPER1 = np.array([10, 255, 255])
RED_LOWER2 = np.array([160, 100, 100])
RED_UPPER2 = np.array([180, 255, 255])

GREEN_LOWER = np.array([40, 50, 50])
GREEN_UPPER = np.array([80, 255, 255])

MAGENTA_LOWER = np.array([140, 100, 100])
MAGENTA_UPPER = np.array([160, 255, 255])


@dataclass
class ColorBlob:
    """A detected colored region in the camera frame."""

    color: str      # "red", "green", "magenta"
    angle: float    # Degrees from center (-60 to +60 for 120 FOV)
    x: int          # Pixel x (center of blob)
    y: int          # Pixel y (center of blob)
    width: int      # Bounding box width in pixels
    height: int     # Bounding box height in pixels
    area: int       # Contour area in pixels


class Camera:
    """
    Captures frames and detects colored objects in background thread.

    Usage:
        camera = Camera()
        camera.start()

        blobs = camera.get_blobs()
        for blob in blobs:
            print(f"{blob.color} at {blob.angle:.0f} degrees")

        jpeg = camera.get_jpeg_frame()       # Frame with bounding boxes
        mask = camera.get_jpeg_mask("red")   # Red color mask

        camera.stop()
    """

    def __init__(self):
        self._cap: cv2.VideoCapture | None = None
        self._running = False
        self._thread: threading.Thread | None = None
        self._lock = threading.Lock()

        # Latest frame and detections
        self._frame: np.ndarray | None = None
        self._blobs: list[ColorBlob] = []

    @property
    def is_running(self) -> bool:
        return self._running

    def start(self) -> bool:
        """Open camera and start capturing in background."""
        if self._running:
            return True

        self._cap = cv2.VideoCapture(CAMERA_INDEX)
        self._cap.set(cv2.CAP_PROP_FRAME_WIDTH, CAMERA_WIDTH)
        self._cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAMERA_HEIGHT)

        if not self._cap.isOpened():
            print("ERROR: Failed to open camera")
            return False

        self._running = True
        self._thread = threading.Thread(target=self._capture_loop, daemon=True)
        self._thread.start()

        print(f"Camera started: {CAMERA_WIDTH}x{CAMERA_HEIGHT}")
        return True

    def stop(self):
        """Stop camera capture."""
        self._running = False

        if self._thread:
            self._thread.join(timeout=2.0)
            self._thread = None

        if self._cap:
            self._cap.release()
            self._cap = None

        print("Camera stopped")

    def get_frame(self) -> np.ndarray | None:
        """Get latest frame as numpy array (BGR format)."""
        with self._lock:
            if self._frame is not None:
                return self._frame.copy()
            return None

    def get_blobs(self) -> list[ColorBlob]:
        """Get all detected color blobs."""
        with self._lock:
            return self._blobs.copy()

    def get_red_blobs(self) -> list[ColorBlob]:
        """Get red blobs (pillars to pass on RIGHT)."""
        return [b for b in self.get_blobs() if b.color == "red"]

    def get_green_blobs(self) -> list[ColorBlob]:
        """Get green blobs (pillars to pass on LEFT)."""
        return [b for b in self.get_blobs() if b.color == "green"]

    def get_jpeg_frame(self, draw_boxes: bool = True, quality: int = 80) -> bytes | None:
        """Get frame as JPEG, optionally with bounding boxes drawn.

        Args:
            draw_boxes: If True, draw rectangles around detected blobs.
            quality: JPEG compression quality (0-100).
        """
        with self._lock:
            if self._frame is None:
                return None
            frame = self._frame.copy()
            blobs = self._blobs.copy()

        if draw_boxes:
            # BGR colors for drawing (not HSV!)
            colors = {
                "red": (0, 0, 255),       # Red in BGR
                "green": (0, 255, 0),     # Green in BGR
                "magenta": (255, 0, 255), # Magenta in BGR
            }
            for blob in blobs:
                bgr = colors.get(blob.color, (255, 255, 255))

                # Calculate top-left corner from center
                x = blob.x - blob.width // 2
                y = blob.y - blob.height // 2

                # Draw bounding box
                cv2.rectangle(frame, (x, y), (x + blob.width, y + blob.height), bgr, 2)

                # Draw label
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
        """Get binary color mask as JPEG bytes.

        Shows white pixels where the color is detected, black everywhere else.

        Args:
            color: "red", "green", or "magenta"
            quality: JPEG compression quality (0-100)
        """
        with self._lock:
            if self._frame is None:
                return None
            frame = self._frame.copy()

        # Convert BGR -> HSV
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Create binary mask for the requested color
        if color == "red":
            # Red wraps around hue, need two ranges combined with OR
            mask1 = cv2.inRange(hsv, RED_LOWER1, RED_UPPER1)
            mask2 = cv2.inRange(hsv, RED_LOWER2, RED_UPPER2)
            mask = cv2.bitwise_or(mask1, mask2)
        elif color == "green":
            mask = cv2.inRange(hsv, GREEN_LOWER, GREEN_UPPER)
        elif color == "magenta":
            mask = cv2.inRange(hsv, MAGENTA_LOWER, MAGENTA_UPPER)
        else:
            return None

        ret, jpeg = cv2.imencode(".jpg", mask, [cv2.IMWRITE_JPEG_QUALITY, quality])
        if not ret:
            return None
        return jpeg.tobytes()

    # ── Private methods ────────────────────────────────────────────

    def _capture_loop(self):
        """Background thread: grab frames and run detection."""
        while self._running:
            ret, frame = self._cap.read()
            if not ret:
                continue

            # Run color detection on this frame
            blobs = self._detect_blobs(frame)

            # Store results safely
            with self._lock:
                self._frame = frame
                self._blobs = blobs

    def _detect_blobs(self, frame: np.ndarray) -> list[ColorBlob]:
        """Detect all colored blobs in a frame."""
        # Convert BGR (camera format) to HSV (better for color detection)
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        blobs = []

        # Detect red (needs two ranges because red wraps around hue 0/180)
        mask_red1 = cv2.inRange(hsv, RED_LOWER1, RED_UPPER1)
        mask_red2 = cv2.inRange(hsv, RED_LOWER2, RED_UPPER2)
        mask_red = cv2.bitwise_or(mask_red1, mask_red2)
        blobs.extend(self._find_blobs(mask_red, "red"))

        # Detect green (single range, sits in middle of hue spectrum)
        mask_green = cv2.inRange(hsv, GREEN_LOWER, GREEN_UPPER)
        blobs.extend(self._find_blobs(mask_green, "green"))

        # Detect magenta (single range)
        mask_magenta = cv2.inRange(hsv, MAGENTA_LOWER, MAGENTA_UPPER)
        blobs.extend(self._find_blobs(mask_magenta, "magenta"))

        return blobs

    def _find_blobs(self, mask: np.ndarray, color: str) -> list[ColorBlob]:
        """Find blobs in a binary mask (white = detected, black = not)."""
        blobs = []

        # Clean up the mask: erode removes tiny noise, dilate fills small gaps
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.erode(mask, kernel, iterations=1)
        mask = cv2.dilate(mask, kernel, iterations=2)

        # Find contours (outlines of white regions)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        for contour in contours:
            area = cv2.contourArea(contour)
            if area < MIN_AREA:
                continue  # Too small, probably noise

            # Get bounding rectangle
            x, y, w, h = cv2.boundingRect(contour)
            center_x = x + w // 2
            center_y = y + h // 2

            # Convert pixel X position to angle from center
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
        """Convert X pixel position to angle from camera center.

        With 120 degree FOV and 640px width:
        - Pixel 0   -> -60 degrees (far left)
        - Pixel 320 ->   0 degrees (center)
        - Pixel 640 -> +60 degrees (far right)
        """
        normalized = (pixel_x - CAMERA_WIDTH / 2) / (CAMERA_WIDTH / 2)
        return normalized * (CAMERA_FOV / 2)
