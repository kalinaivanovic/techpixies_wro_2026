"""
WorldState visualizer - Renders perception output as annotated OpenCV images.

Produces two views:
- Bird's eye: LIDAR points + wall lines + pillar markers + corner arrows
- Camera: Raw frame + confirmed/unmatched blob boxes + distance overlays
"""

from __future__ import annotations

import math
import cv2
import numpy as np

from sensors.camera import ColorBlob
from .world_state import WorldState, Pillar


# Colors (BGR)
_WHITE = (255, 255, 255)
_GRAY = (120, 120, 120)
_DARK_GRAY = (60, 60, 60)
_CYAN = (255, 255, 0)
_RED = (0, 0, 255)
_GREEN = (0, 220, 0)
_MAGENTA = (255, 0, 255)
_YELLOW = (0, 220, 220)
_ORANGE = (0, 140, 255)

_PILLAR_COLORS = {"red": _RED, "green": _GREEN}


class WorldStateVisualizer:
    """Renders WorldState as annotated OpenCV images."""

    def __init__(self, size: int = 500, max_range: int = 3000):
        self.size = size
        self.max_range = max_range

    # ── Public API ──────────────────────────────────────────────

    def render_birdseye(self, world: WorldState | None, scan: dict[int, float], state_name: str = "", lap: int = 0, direction: str = "") -> bytes:
        """Render bird's eye view with WorldState annotations.

        Args:
            world: Current WorldState (None handled gracefully).
            scan: Raw LIDAR scan dict (angle -> distance mm).
            state_name: Current robot state name for overlay.
            lap: Current lap number.
            direction: "CW" or "CCW".

        Returns:
            JPEG bytes.
        """
        size = self.size
        center = size // 2
        scale = (size // 2) / self.max_range
        image = np.zeros((size, size, 3), dtype=np.uint8)

        # Range rings
        for r_mm in range(500, self.max_range + 1, 500):
            r_px = int(r_mm * scale)
            cv2.circle(image, (center, center), r_px, (30, 30, 30), 1)

        # Draw raw LIDAR points
        for angle_deg, distance in scan.items():
            if distance > self.max_range:
                continue
            x, y = self._polar_to_px(angle_deg, distance, center, scale)
            if 0 <= x < size and 0 <= y < size:
                cv2.circle(image, (x, y), 2, _CYAN, -1)

        # Robot marker
        cv2.drawMarker(
            image, (center, center), (0, 200, 0),
            cv2.MARKER_TRIANGLE_UP, 14, 2,
        )

        if world is None:
            self._put_centered(image, "No WorldState", size // 2, 30, _GRAY)
            return self._encode(image)

        # Wall distance lines
        self._draw_wall_lines(image, world, center, scale)

        # Confirmed pillars
        for pillar in world.pillars:
            self._draw_pillar_marker(image, pillar, center, scale)

        # Corner indicator
        if world.corner_ahead:
            self._draw_corner_arrow(image, world.corner_ahead)

        # Parking indicator
        if world.parking_marker is not None:
            cv2.putText(
                image, f"PARK {world.parking_marker:.0f}mm",
                (10, size - 15), cv2.FONT_HERSHEY_SIMPLEX,
                0.5, _MAGENTA, 1,
            )

        # Text overlay
        self._draw_text_overlay(
            image, state_name, lap, direction, world.encoder_pos,
        )

        return self._encode(image)

    def render_camera(self, world: WorldState | None, frame: np.ndarray | None, blobs: list[ColorBlob], state_name: str = "", lap: int = 0) -> bytes:
        """Render camera frame with fusion annotations.

        Args:
            world: Current WorldState (None handled gracefully).
            frame: Raw BGR camera frame (None handled).
            blobs: All detected color blobs from camera.
            state_name: Current robot state name.
            lap: Current lap number.

        Returns:
            JPEG bytes.
        """
        if frame is None:
            placeholder = np.zeros((360, 480, 3), dtype=np.uint8)
            self._put_centered(placeholder, "No Camera", 240, 180, _GRAY)
            return self._encode(placeholder)

        display = frame.copy()
        confirmed_angles = set()

        if world is not None:
            # Collect angles of confirmed pillars for matching
            for pillar in world.pillars:
                confirmed_angles.add(pillar.angle)

            # Draw confirmed pillar boxes
            for pillar in world.pillars:
                blob = self._find_matching_blob(pillar, blobs)
                if blob:
                    self._draw_confirmed_blob(display, blob, pillar)

            # Draw wall distance overlay
            self._draw_wall_overlay(display, world)

            # Corner warning
            if world.corner_ahead:
                cv2.putText(
                    display,
                    f"CORNER {world.corner_ahead}",
                    (display.shape[1] // 2 - 80, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, _ORANGE, 2,
                )

        # Draw unmatched blobs (camera saw it, fusion didn't confirm)
        for blob in blobs:
            if not self._is_blob_confirmed(blob, world):
                self._draw_unmatched_blob(display, blob)

        # State/lap text
        h = display.shape[0]
        label = state_name if state_name else "---"
        if lap:
            label += f"  Lap {lap}"
        cv2.putText(
            display, label,
            (8, h - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, _WHITE, 1,
        )

        return self._encode(display)

    # ── Bird's eye helpers ──────────────────────────────────────

    @staticmethod
    def _polar_to_px(angle_deg: float, distance: float, center: int, scale: float) -> tuple[int, int]:
        """Convert polar (angle, distance) to pixel coords."""
        angle_rad = math.radians(angle_deg)
        x = int(center + distance * math.sin(angle_rad) * scale)
        y = int(center - distance * math.cos(angle_rad) * scale)
        return x, y

    def _draw_wall_lines(self, image: np.ndarray, world: WorldState, center: int, scale: float) -> None:
        walls = world.walls
        size = self.size
        line_len = size // 3  # Length of the wall indicator line

        # Left wall (270 deg)
        if walls.left_distance is not None:
            lx = int(center - walls.left_distance * scale)
            y1 = center - line_len // 2
            y2 = center + line_len // 2
            lx = max(0, min(lx, size - 1))
            cv2.line(image, (lx, y1), (lx, y2), _YELLOW, 2)
            cv2.putText(
                image, f"L {walls.left_distance:.0f}",
                (lx + 4, y1 - 6), cv2.FONT_HERSHEY_SIMPLEX,
                0.35, _YELLOW, 1,
            )

        # Right wall (90 deg)
        if walls.right_distance is not None:
            rx = int(center + walls.right_distance * scale)
            y1 = center - line_len // 2
            y2 = center + line_len // 2
            rx = max(0, min(rx, size - 1))
            cv2.line(image, (rx, y1), (rx, y2), _YELLOW, 2)
            cv2.putText(
                image, f"R {walls.right_distance:.0f}",
                (rx + 4, y1 - 6), cv2.FONT_HERSHEY_SIMPLEX,
                0.35, _YELLOW, 1,
            )

        # Front wall (0 deg)
        if walls.front_distance is not None:
            fy = int(center - walls.front_distance * scale)
            x1 = center - line_len // 2
            x2 = center + line_len // 2
            fy = max(0, min(fy, size - 1))
            cv2.line(image, (x1, fy), (x2, fy), _YELLOW, 2)
            cv2.putText(
                image, f"F {walls.front_distance:.0f}",
                (x2 + 4, fy - 4), cv2.FONT_HERSHEY_SIMPLEX,
                0.35, _YELLOW, 1,
            )

    def _draw_pillar_marker(self, image: np.ndarray, pillar: Pillar, center: int, scale: float) -> None:
        color = _PILLAR_COLORS.get(pillar.color, _WHITE)

        # Convert pillar angle: camera convention (positive=right) to LIDAR convention
        # Camera angle: 0=forward, positive=right
        # _polar_to_px expects LIDAR convention: 0=forward, 90=right
        lidar_angle = pillar.angle  # already in signed degrees from center
        x, y = self._polar_to_px(lidar_angle, pillar.distance, center, scale)

        # Filled circle
        cv2.circle(image, (x, y), 10, color, -1)
        cv2.circle(image, (x, y), 10, _WHITE, 1)

        # Label
        label = f"{pillar.color.upper()} {pillar.distance:.0f}mm"
        cv2.putText(
            image, label,
            (x + 14, y + 4), cv2.FONT_HERSHEY_SIMPLEX,
            0.4, color, 1,
        )

    def _draw_corner_arrow(self, image: np.ndarray, direction: str) -> None:
        size = self.size
        arrow_y = 40
        center_x = size // 2

        if direction == "LEFT":
            start = (center_x + 30, arrow_y)
            end = (center_x - 30, arrow_y)
        else:
            start = (center_x - 30, arrow_y)
            end = (center_x + 30, arrow_y)

        cv2.arrowedLine(image, start, end, _ORANGE, 3, tipLength=0.4)
        cv2.putText(
            image, f"CORNER {direction}",
            (center_x - 55, arrow_y + 20),
            cv2.FONT_HERSHEY_SIMPLEX, 0.45, _ORANGE, 1,
        )

    def _draw_text_overlay(self, image: np.ndarray, state_name: str, lap: int, direction: str, encoder: int) -> None:
        lines = []
        if state_name:
            lines.append(state_name)
        if direction:
            lines.append(f"Dir: {direction}")
        if lap:
            lines.append(f"Lap: {lap}")
        lines.append(f"Enc: {encoder}")

        y = 18
        for line in lines:
            cv2.putText(
                image, line, (8, y),
                cv2.FONT_HERSHEY_SIMPLEX, 0.45, _WHITE, 1,
            )
            y += 18

    # ── Camera helpers ──────────────────────────────────────────

    @staticmethod
    def _find_matching_blob(pillar: Pillar, blobs: list[ColorBlob]) -> ColorBlob | None:
        """Find the camera blob closest in angle to a confirmed pillar."""
        best = None
        best_diff = float("inf")
        for blob in blobs:
            if blob.color != pillar.color:
                continue
            diff = abs(blob.angle - pillar.angle)
            if diff < best_diff:
                best_diff = diff
                best = blob
        return best

    @staticmethod
    def _is_blob_confirmed(blob: ColorBlob, world: WorldState | None) -> bool:
        if world is None:
            return False
        for pillar in world.pillars:
            if pillar.color == blob.color and abs(pillar.angle - blob.angle) < 15:
                return True
        return False

    @staticmethod
    def _draw_confirmed_blob(image: np.ndarray, blob: ColorBlob, pillar: Pillar) -> None:
        color = _PILLAR_COLORS.get(blob.color, _WHITE)
        x = blob.x - blob.width // 2
        y = blob.y - blob.height // 2

        # Thick box
        cv2.rectangle(
            image, (x, y), (x + blob.width, y + blob.height), color, 3,
        )

        # Distance label
        label = f"{pillar.distance:.0f}mm"
        cv2.putText(
            image, label, (x, y - 8),
            cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 2,
        )

        # Checkmark
        cx = x + blob.width + 6
        cy = y + 12
        cv2.line(image, (cx, cy), (cx + 5, cy + 8), _GREEN, 2)
        cv2.line(image, (cx + 5, cy + 8), (cx + 14, cy - 4), _GREEN, 2)

    @staticmethod
    def _draw_unmatched_blob(image: np.ndarray, blob: ColorBlob) -> None:
        color = _PILLAR_COLORS.get(blob.color, _GRAY)
        dimmed = tuple(c // 2 for c in color)
        x = blob.x - blob.width // 2
        y = blob.y - blob.height // 2

        cv2.rectangle(
            image, (x, y), (x + blob.width, y + blob.height), dimmed, 1,
        )
        cv2.putText(
            image, "?", (x + blob.width // 2 - 5, y - 6),
            cv2.FONT_HERSHEY_SIMPLEX, 0.5, dimmed, 1,
        )

    @staticmethod
    def _draw_wall_overlay(image: np.ndarray, world: WorldState) -> None:
        """Draw wall distance text in camera frame corners."""
        h, w = image.shape[:2]
        walls = world.walls

        entries = []
        if walls.left_distance is not None:
            entries.append((f"L:{walls.left_distance:.0f}", (8, h // 2)))
        if walls.right_distance is not None:
            entries.append((f"R:{walls.right_distance:.0f}", (w - 80, h // 2)))
        if walls.front_distance is not None:
            entries.append((f"F:{walls.front_distance:.0f}", (w // 2 - 30, 55)))

        for text, pos in entries:
            # Dark background for readability
            (tw, th), _ = cv2.getTextSize(
                text, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1,
            )
            px, py = pos
            cv2.rectangle(
                image, (px - 2, py - th - 4), (px + tw + 2, py + 4),
                (0, 0, 0), -1,
            )
            cv2.putText(
                image, text, pos,
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, _YELLOW, 1,
            )

    # ── Utilities ───────────────────────────────────────────────

    @staticmethod
    def _put_centered(image: np.ndarray, text: str, cx: int, cy: int, color: tuple) -> None:
        (tw, th), _ = cv2.getTextSize(
            text, cv2.FONT_HERSHEY_SIMPLEX, 0.7, 1,
        )
        cv2.putText(
            image, text, (cx - tw // 2, cy + th // 2),
            cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 1,
        )

    @staticmethod
    def _encode(image: np.ndarray, quality: int = 80) -> bytes:
        ret, jpeg = cv2.imencode(
            ".jpg", image, [cv2.IMWRITE_JPEG_QUALITY, quality],
        )
        if not ret:
            return b""
        return jpeg.tobytes()
