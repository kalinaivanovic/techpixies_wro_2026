"""
Track map - Accumulated knowledge from lap 1.

Since WRO track layout is randomized, the robot learns
the track during lap 1 and uses that knowledge for laps 2-3.
"""

from __future__ import annotations

import logging
from dataclasses import dataclass, field
from .world_state import WorldState

logger = logging.getLogger(__name__)


@dataclass
class Corner:
    """Recorded corner location."""

    encoder_pos: int
    direction: str  # "LEFT" or "RIGHT"


@dataclass
class Section:
    """Track section with measured corridor width."""

    start_encoder: int
    end_encoder: int
    width: float  # Average corridor width in mm


@dataclass
class PillarRecord:
    """Recorded pillar location."""

    encoder_pos: int
    color: str  # "red" or "green"
    side: str  # "left" or "right" of track center
    angle: float = 0.0


class TrackMap:
    """
    Accumulates track knowledge during lap 1.

    Records:
    - Direction (CW or CCW)
    - Corner positions
    - Corridor widths per section
    - Pillar locations
    - Parking zone location

    Usage:
        track_map = TrackMap()

        # During lap 1, call update() each frame:
        track_map.update(world_state)

        # Query for navigation:
        next_corner = track_map.get_next_corner(encoder)
    """

    def __init__(self, corner_tolerance: int = 100, pillar_tolerance: int = 50):
        self.corner_tolerance = corner_tolerance
        self.pillar_tolerance = pillar_tolerance

        # Track data
        self.direction: str | None = None  # "CW" or "CCW"
        self.corners: list[Corner] = []
        self.sections: list[Section] = []
        self.pillars: list[PillarRecord] = []
        self.parking_zone: tuple[int, int] | None = None
        self.lap_length: int | None = None

        # Mapping state
        self._first_lap = True
        self._lap_start: int | None = None
        self._last_corner_enc = -1000
        self._section_start: int | None = None
        self._section_widths: list[float] = []

    @property
    def is_first_lap(self) -> bool:
        return self._first_lap

    @property
    def corner_count(self) -> int:
        return len(self.corners)

    def update(self, world: WorldState) -> None:
        """Update map with current perception. Call each frame during lap 1."""
        if not self._first_lap:
            return

        encoder = world.encoder_pos

        # Initialize
        if self._lap_start is None:
            self._lap_start = encoder
            self._section_start = encoder
            logger.info(f"TrackMap: Starting at encoder {encoder}")

        # Detect direction from first corner
        if self.direction is None and world.corner_ahead:
            self.direction = "CW" if world.corner_ahead == "RIGHT" else "CCW"
            logger.info(f"TrackMap: Direction = {self.direction}")

        # Record corners
        self._update_corners(world)

        # Record corridor widths
        self._update_sections(world)

        # Record pillars
        self._update_pillars(world)

        # Record parking zone
        self._update_parking(world)

        # Check lap completion (4 corners = 1 lap)
        if len(self.corners) >= 4 and self.lap_length is None:
            self._finalize_lap(encoder)

    def _update_corners(self, world: WorldState) -> None:
        if world.corner_ahead is None:
            return

        encoder = world.encoder_pos
        if abs(encoder - self._last_corner_enc) < self.corner_tolerance:
            return  # Duplicate

        corner = Corner(encoder_pos=encoder, direction=world.corner_ahead)
        self.corners.append(corner)
        self._last_corner_enc = encoder
        logger.info(f"TrackMap: Corner {corner.direction} at {encoder}")

        # Finalize section
        self._finalize_section(encoder)
        self._section_start = encoder
        self._section_widths = []

    def _update_sections(self, world: WorldState) -> None:
        if world.corridor_width is not None:
            self._section_widths.append(world.corridor_width)

    def _finalize_section(self, end_encoder: int) -> None:
        if self._section_start is None or not self._section_widths:
            return

        avg_width = sum(self._section_widths) / len(self._section_widths)
        section = Section(
            start_encoder=self._section_start,
            end_encoder=end_encoder,
            width=avg_width,
        )
        self.sections.append(section)
        logger.info(f"TrackMap: Section width={avg_width:.0f}mm")

    def _update_pillars(self, world: WorldState) -> None:
        for pillar in world.pillars:
            if self._is_new_pillar(world.encoder_pos, pillar.color):
                side = "right" if pillar.angle > 0 else "left"
                record = PillarRecord(
                    encoder_pos=world.encoder_pos,
                    color=pillar.color,
                    side=side,
                    angle=pillar.angle,
                )
                self.pillars.append(record)
                logger.info(f"TrackMap: Pillar {pillar.color} at {world.encoder_pos}")

    def _is_new_pillar(self, encoder: int, color: str) -> bool:
        for p in self.pillars:
            if p.color == color and abs(p.encoder_pos - encoder) < self.pillar_tolerance:
                return False
        return True

    def _update_parking(self, world: WorldState) -> None:
        if world.parking_marker is not None and self.parking_zone is None:
            encoder = world.encoder_pos
            self.parking_zone = (encoder - 100, encoder + 300)
            logger.info(f"TrackMap: Parking zone at {encoder}")

    def _finalize_lap(self, encoder: int) -> None:
        if self._lap_start is None:
            return

        self.lap_length = encoder - self._lap_start
        self._first_lap = False
        logger.info(f"TrackMap: Lap complete! Length={self.lap_length}")
        logger.info(
            f"TrackMap: {len(self.corners)} corners, "
            f"{len(self.sections)} sections, "
            f"{len(self.pillars)} pillars"
        )

    # =========================================================================
    # Query methods
    # =========================================================================

    def get_next_corner(self, encoder: int) -> tuple[int, str] | None:
        """Get (distance, direction) to next corner, or None."""
        if not self.corners:
            return None

        if self.lap_length is None:
            # First lap - look ahead
            for corner in self.corners:
                if corner.encoder_pos > encoder:
                    return (corner.encoder_pos - encoder, corner.direction)
            return None

        # Subsequent laps - wrap around
        normalized = encoder % self.lap_length
        for corner in self.corners:
            corner_norm = corner.encoder_pos % self.lap_length
            if corner_norm > normalized:
                return (corner_norm - normalized, corner.direction)

        # Wrap to first corner
        if self.corners:
            first = self.corners[0]
            dist = (self.lap_length - normalized) + (first.encoder_pos % self.lap_length)
            return (dist, first.direction)

        return None

    def get_expected_pillars(self, encoder: int, lookahead: int = 500) -> list[PillarRecord]:
        """Get pillars expected in next `lookahead` encoder ticks."""
        result = []

        if self.lap_length is None:
            for p in self.pillars:
                dist = p.encoder_pos - encoder
                if 0 <= dist <= lookahead:
                    result.append(p)
        else:
            normalized = encoder % self.lap_length
            for p in self.pillars:
                p_norm = p.encoder_pos % self.lap_length
                if p_norm >= normalized:
                    dist = p_norm - normalized
                else:
                    dist = (self.lap_length - normalized) + p_norm
                if 0 <= dist <= lookahead:
                    result.append(p)

        return result

    def get_section_width(self, encoder: int) -> float | None:
        """Get expected corridor width at current position."""
        if self.lap_length is None:
            for section in self.sections:
                if section.start_encoder <= encoder <= section.end_encoder:
                    return section.width
            return None

        normalized = encoder % self.lap_length
        for section in self.sections:
            start = section.start_encoder % self.lap_length
            end = section.end_encoder % self.lap_length
            if start <= normalized <= end:
                return section.width

        return None

    def get_distance_to_parking(self, encoder: int) -> int | None:
        """Get distance to parking zone, or None."""
        if self.parking_zone is None:
            return None

        start, _ = self.parking_zone

        if self.lap_length is None:
            dist = start - encoder
            return dist if dist > 0 else None

        normalized = encoder % self.lap_length
        start_norm = start % self.lap_length

        if start_norm >= normalized:
            return start_norm - normalized
        return (self.lap_length - normalized) + start_norm

    def should_prepare_parking(self, encoder: int, lap_count: int, distance: int = 500) -> bool:
        """Check if should start parking preparation (lap 3 only)."""
        if lap_count < 3:
            return False

        dist = self.get_distance_to_parking(encoder)
        return dist is not None and dist <= distance
