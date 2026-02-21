"""
Camera - capture frames and detect colored objects.

Stage 3: Now reads HSV ranges from Parameters instead of hardcoded constants.

What changed from Stage 2:
  - Constructor takes a `params` object
  - _range() method builds HSV arrays from params (not constants)
  - _detect_blobs() reads min_area from params
  - If you change params at runtime, the NEXT frame uses new values!

This is possible because Python reads self.params.red_h_min each frame.
There is no caching. So the web server can update params between frames,
and the camera thread sees the new values immediately.

Key insight: the Camera doesn't OWN the parameters. It just holds a
REFERENCE to the same object that the web server modifies.

    params = Parameters()
    camera = Camera(params)   # camera.params IS the same object
    params.red_h_min = 5      # camera sees the change next frame
"""

import threading
import time
from dataclasses import dataclass

import cv2
import numpy as np

from params import Parameters

# Camera settings
CAMERA_INDEX = 0
CAMERA_WIDTH = 640
CAMERA_HEIGHT = 480
CAMERA_FOV = 120  # degrees (wide-angle lens)


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
    Captures frames and detects colored objects.

    Reads all detection settings from a shared Parameters object,
    so HSV ranges can be tuned at runtime without restarting.

    Usage:
        params = Parameters.load()
        camera = Camera(params)
        camera.start()

        blobs = camera.get_blobs()
        jpeg = camera.get_jpeg_frame()

        # Change a parameter at runtime:
        params.red_h_min = 5
        # Next frame will use the new value!

        camera.stop()
    """

    def __init__(self, params: Parameters):
        # Store REFERENCE to shared params (not a copy!)
        self.params = params

        self._cap: cv2.VideoCapture | None = None
        self._running = False
        self._thread: threading.Thread | None = None
        self._lock = threading.Lock()

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

    def get_blobs(self) -> list[ColorBlob]:
        """Get all detected color blobs."""
        with self._lock:
            return self._blobs.copy()

    def get_frame(self) -> np.ndarray | None:
        """Get latest frame (BGR format)."""
        with self._lock:
            if self._frame is not None:
                return self._frame.copy()
            return None

    def get_jpeg_frame(self, quality: int = 80) -> bytes | None:
        """Get frame with bounding boxes as JPEG bytes."""
        with self._lock:
            if self._frame is None:
                return None
            frame = self._frame.copy()
            blobs = self._blobs.copy()

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
                frame, f"{blob.color} {blob.angle:.0f}deg",
                (x, y - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.5, bgr, 1,
            )

        ret, jpeg = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, quality])
        return jpeg.tobytes() if ret else None

    def get_jpeg_mask(self, color: str, quality: int = 80) -> bytes | None:
        """Get binary color mask as JPEG bytes."""
        with self._lock:
            if self._frame is None:
                return None
            frame = self._frame.copy()

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        if color == "red":
            lower1, upper1 = self._range("red1")
            lower2, upper2 = self._range("red2")
            mask = cv2.bitwise_or(
                cv2.inRange(hsv, lower1, upper1),
                cv2.inRange(hsv, lower2, upper2),
            )
        elif color == "green":
            lower, upper = self._range("green")
            mask = cv2.inRange(hsv, lower, upper)
        elif color == "magenta":
            lower, upper = self._range("magenta")
            mask = cv2.inRange(hsv, lower, upper)
        else:
            return None

        ret, jpeg = cv2.imencode(".jpg", mask, [cv2.IMWRITE_JPEG_QUALITY, quality])
        return jpeg.tobytes() if ret else None

    # ── Private methods ──────────────────────────────────────────

    def _range(self, color: str):
        """Build (lower, upper) HSV numpy arrays from params.

        THIS is the key difference from Stage 2.
        Stage 2: used hardcoded constants like RED_LOWER1 = np.array([0, 100, 100])
        Stage 3: reads from self.params every time this is called.

        Since _detect_blobs() calls _range() every frame, any changes
        to params take effect on the very next frame.
        """
        p = self.params
        if color == "red1":
            return (np.array([p.red_h_min, p.red_s_min, p.red_v_min]),
                    np.array([p.red_h_max, p.red_s_max, p.red_v_max]))
        if color == "red2":
            return (np.array([p.red_h_min2, p.red_s_min2, p.red_v_min2]),
                    np.array([p.red_h_max2, p.red_s_max2, p.red_v_max2]))
        if color == "green":
            return (np.array([p.green_h_min, p.green_s_min, p.green_v_min]),
                    np.array([p.green_h_max, p.green_s_max, p.green_v_max]))
        # magenta
        return (np.array([p.magenta_h_min, p.magenta_s_min, p.magenta_v_min]),
                np.array([p.magenta_h_max, p.magenta_s_max, p.magenta_v_max]))

    def _capture_loop(self):
        """Background thread: grab frames and run detection."""
        while self._running:
            ret, frame = self._cap.read()
            if not ret:
                continue
            blobs = self._detect_blobs(frame)
            with self._lock:
                self._frame = frame
                self._blobs = blobs

    def _detect_blobs(self, frame: np.ndarray) -> list[ColorBlob]:
        """Detect colored blobs using current params."""
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        blobs = []

        # Read min_area from params (not a constant!)
        min_area = self.params.min_area

        # Red (two ranges)
        rl1, ru1 = self._range("red1")
        rl2, ru2 = self._range("red2")
        mask_red = cv2.bitwise_or(
            cv2.inRange(hsv, rl1, ru1),
            cv2.inRange(hsv, rl2, ru2),
        )
        blobs.extend(self._find_blobs(mask_red, "red", min_area))

        # Green
        gl, gu = self._range("green")
        blobs.extend(self._find_blobs(cv2.inRange(hsv, gl, gu), "green", min_area))

        # Magenta
        ml, mu = self._range("magenta")
        blobs.extend(self._find_blobs(cv2.inRange(hsv, ml, mu), "magenta", min_area))

        return blobs

    def _find_blobs(self, mask: np.ndarray, color: str, min_area: int) -> list[ColorBlob]:
        """Find blobs in a binary mask."""
        blobs = []
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.erode(mask, kernel, iterations=1)
        mask = cv2.dilate(mask, kernel, iterations=2)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        for contour in contours:
            area = cv2.contourArea(contour)
            if area < min_area:
                continue
            x, y, w, h = cv2.boundingRect(contour)
            center_x = x + w // 2
            center_y = y + h // 2
            angle = self._pixel_to_angle(center_x)
            blobs.append(ColorBlob(color, angle, center_x, center_y, w, h, int(area)))

        return blobs

    def _pixel_to_angle(self, pixel_x: int) -> float:
        """Convert X pixel position to angle from center."""
        normalized = (pixel_x - CAMERA_WIDTH / 2) / (CAMERA_WIDTH / 2)
        return normalized * (CAMERA_FOV / 2)
