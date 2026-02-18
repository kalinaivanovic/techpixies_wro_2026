"""
Camera - connect to camera and capture frames.

Stage 1: Just capture, no color detection.
"""

import threading
import time

import cv2
import numpy as np

# Camera settings (hardcoded so you can see the actual values)
CAMERA_INDEX = 0       # 0 = default camera
CAMERA_WIDTH = 640     # pixels
CAMERA_HEIGHT = 480    # pixels


class Camera:
    """
    Captures frames from camera in a background thread.

    Usage:
        camera = Camera()
        camera.start()

        jpeg = camera.get_jpeg_frame()   # JPEG bytes for streaming

        camera.stop()
    """

    def __init__(self):
        # OpenCV video capture object (None until start() is called)
        self._cap: cv2.VideoCapture | None = None

        # Flag to control the background thread
        self._running = False

        # Background thread that continuously grabs frames
        self._thread: threading.Thread | None = None

        # Lock to safely share data between threads
        self._lock = threading.Lock()

        # Latest captured frame (None until first frame arrives)
        self._frame: np.ndarray | None = None

    @property
    def is_running(self) -> bool:
        return self._running

    def start(self) -> bool:
        """Open camera and start capturing in background."""
        if self._running:
            return True

        # Open camera
        self._cap = cv2.VideoCapture(CAMERA_INDEX)
        self._cap.set(cv2.CAP_PROP_FRAME_WIDTH, CAMERA_WIDTH)
        self._cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAMERA_HEIGHT)

        if not self._cap.isOpened():
            print("ERROR: Failed to open camera")
            return False

        # Start background thread
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

    def get_jpeg_frame(self, quality: int = 80) -> bytes | None:
        """Get latest frame as JPEG bytes (for streaming to browser).

        Args:
            quality: JPEG compression quality (0-100).

        Returns:
            JPEG bytes, or None if no frame available.
        """
        with self._lock:
            if self._frame is None:
                return None
            frame = self._frame.copy()

        # Encode frame as JPEG
        ret, jpeg = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, quality])
        if not ret:
            return None
        return jpeg.tobytes()

    def _capture_loop(self):
        """Background thread: continuously grab frames from camera."""
        while self._running:
            ret, frame = self._cap.read()
            if not ret:
                continue

            # Store frame safely (lock prevents reading half-written data)
            with self._lock:
                self._frame = frame
