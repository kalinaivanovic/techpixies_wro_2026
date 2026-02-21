"""
Camera - Same as Stage 3 (captures frames, detects colored blobs).

Stage 4 doesn't change the camera itself. The camera's job stays the same:
  - Capture frames
  - Detect colored blobs (red, green, magenta)
  - Report: "I see a RED thing at angle +15 degrees"

What's NEW in Stage 4 happens ABOVE the camera:
  - SensorFusion takes the camera's blobs and ASKS: "Is this a real pillar?"
  - WorldState packages the answer for the decision layer
  - StateMachine decides what to DO about it

Think of it as a chain:
  Camera: "I see red at 15 degrees"  (this file - unchanged)
     ↓
  Fusion: "LIDAR confirms object at 15 degrees, 300mm away. It's a pillar!" (NEW)
     ↓
  WorldState: "There's a RED pillar at 300mm" (NEW)
     ↓
  StateMachine: "RED = pass on RIGHT. Steer left!" (NEW)
"""

import threading
from dataclasses import dataclass

import cv2
import numpy as np

from params import Parameters

CAMERA_INDEX = 0
CAMERA_WIDTH = 640
CAMERA_HEIGHT = 480
CAMERA_FOV = 120


@dataclass
class ColorBlob:
    """A detected colored region in the camera frame."""

    color: str
    angle: float
    x: int
    y: int
    width: int
    height: int
    area: int


class Camera:
    """Captures frames and detects colored objects."""

    def __init__(self, params: Parameters):
        self.params = params
        self._cap = None
        self._running = False
        self._thread = None
        self._lock = threading.Lock()
        self._frame = None
        self._blobs: list[ColorBlob] = []

    @property
    def is_running(self) -> bool:
        return self._running

    def start(self) -> bool:
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
        self._running = False
        if self._thread:
            self._thread.join(timeout=2.0)
        if self._cap:
            self._cap.release()
        print("Camera stopped")

    def get_blobs(self) -> list[ColorBlob]:
        """Get all detected color blobs.

        This is the camera's OUTPUT that upper layers use.
        The blobs list tells you WHAT the camera sees and WHERE.

        Example return:
            [ColorBlob(color="red", angle=15.0, x=370, ...),
             ColorBlob(color="green", angle=-22.5, x=195, ...)]

        The camera does NOT know if these are pillars, t-shirts, or noise.
        That's for the SensorFusion layer to figure out (see fusion.py).
        """
        with self._lock:
            return self._blobs.copy()

    def get_jpeg_frame(self, quality=80):
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

    def get_jpeg_mask(self, color, quality=80):
        with self._lock:
            if self._frame is None:
                return None
            frame = self._frame.copy()
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        if color == "red":
            l1, u1 = self._range("red1")
            l2, u2 = self._range("red2")
            mask = cv2.bitwise_or(
                cv2.inRange(hsv, l1, u1),
                cv2.inRange(hsv, l2, u2),
            )
        elif color == "green":
            l, u = self._range("green")
            mask = cv2.inRange(hsv, l, u)
        elif color == "magenta":
            l, u = self._range("magenta")
            mask = cv2.inRange(hsv, l, u)
        else:
            return None
        ret, jpeg = cv2.imencode(".jpg", mask, [cv2.IMWRITE_JPEG_QUALITY, quality])
        return jpeg.tobytes() if ret else None

    def _range(self, color):
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
        return (np.array([p.magenta_h_min, p.magenta_s_min, p.magenta_v_min]),
                np.array([p.magenta_h_max, p.magenta_s_max, p.magenta_v_max]))

    def _capture_loop(self):
        while self._running:
            ret, frame = self._cap.read()
            if not ret:
                continue
            blobs = self._detect_blobs(frame)
            with self._lock:
                self._frame = frame
                self._blobs = blobs

    def _detect_blobs(self, frame):
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        blobs = []
        min_area = self.params.min_area
        rl1, ru1 = self._range("red1")
        rl2, ru2 = self._range("red2")
        mask_red = cv2.bitwise_or(
            cv2.inRange(hsv, rl1, ru1),
            cv2.inRange(hsv, rl2, ru2),
        )
        blobs.extend(self._find_blobs(mask_red, "red", min_area))
        gl, gu = self._range("green")
        blobs.extend(self._find_blobs(cv2.inRange(hsv, gl, gu), "green", min_area))
        ml, mu = self._range("magenta")
        blobs.extend(self._find_blobs(cv2.inRange(hsv, ml, mu), "magenta", min_area))
        return blobs

    def _find_blobs(self, mask, color, min_area):
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
            cx = x + w // 2
            cy = y + h // 2
            angle = (cx - CAMERA_WIDTH / 2) / (CAMERA_WIDTH / 2) * (CAMERA_FOV / 2)
            blobs.append(ColorBlob(color, angle, cx, cy, w, h, int(area)))
        return blobs
