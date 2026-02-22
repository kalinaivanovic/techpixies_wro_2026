"""
World state - Fused perception output.

WorldState is the high-level understanding of the current environment,
produced by combining LIDAR and camera data.
"""

from __future__ import annotations

from dataclasses import dataclass, field


@dataclass
class WallInfo:
    """Wall distances in cardinal directions."""

    left_distance: float | None = None  # mm
    right_distance: float | None = None  # mm
    front_distance: float | None = None  # mm

    @property
    def corridor_width(self) -> float | None:
        """Total corridor width (left + right)."""
        if self.left_distance is None or self.right_distance is None:
            return None
        return self.left_distance + self.right_distance


@dataclass
class Pillar:
    """
    Confirmed pillar (LIDAR object + camera color match).

    A pillar is confirmed when camera detects a color blob
    at the same angle where LIDAR sees a small object.
    """

    color: str  # "red" or "green"
    angle: float  # Degrees from center (positive = right)
    distance: float  # mm (from LIDAR)

    @property
    def pass_side(self) -> str:
        """Which side to pass: RED = right, GREEN = left."""
        return "right" if self.color == "red" else "left"

    def is_blocking(self, angle_threshold: float = 30.0) -> bool:
        """Check if pillar is roughly ahead (needs avoidance)."""
        return abs(self.angle) < angle_threshold


@dataclass
class WorldState:
    """
    Current instant perception.

    This is the main output of the perception layer,
    used by the decision layer to determine robot behavior.
    """

    timestamp: float
    encoder_pos: int

    # Wall information
    walls: WallInfo = field(default_factory=WallInfo)

    # Detected pillars (confirmed with camera color + LIDAR distance)
    pillars: list[Pillar] = field(default_factory=list)

    # Corner detection
    corner_ahead: str | None = None  # "LEFT", "RIGHT", or None

    # Parking marker detection (distance in mm, None if not visible)
    parking_marker: float | None = None

    @property
    def corridor_width(self) -> float | None:
        """Corridor width from wall info."""
        return self.walls.corridor_width

    @property
    def has_pillars(self) -> bool:
        """Check if any pillars are detected."""
        return len(self.pillars) > 0

    @property
    def closest_pillar(self) -> Pillar | None:
        """Get the closest detected pillar."""
        if not self.pillars:
            return None
        return min(self.pillars, key=lambda p: p.distance)

    @property
    def blocking_pillar(self) -> Pillar | None:
        """Get pillar that requires immediate avoidance."""
        blocking = [p for p in self.pillars if p.is_blocking()]
        if not blocking:
            return None
        return min(blocking, key=lambda p: p.distance)

    @property
    def is_corner_approaching(self) -> bool:
        """Check if a corner is detected ahead."""
        return self.corner_ahead is not None

    @property
    def is_parking_visible(self) -> bool:
        """Check if parking marker is visible."""
        return self.parking_marker is not None
