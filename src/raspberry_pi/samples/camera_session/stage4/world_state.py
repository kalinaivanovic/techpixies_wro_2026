"""
WorldState - The robot's understanding of the world.

Stage 4: This is the KEY new concept.

The camera sees "red blob at 15 degrees".
The LIDAR sees "object at 15 degrees, 300mm away".
Neither sensor alone tells us: "There's a RED PILLAR at 300mm."

WorldState is the COMBINED understanding:
  - Walls: how far are the walls? (LIDAR)
  - Pillars: confirmed colored objects (camera + LIDAR matched)
  - Corner: is there a corner ahead? (LIDAR)

Think of it like this:

    Camera alone:  "I see something red"  (could be a t-shirt!)
    LIDAR alone:   "Something 300mm away"  (could be a wall!)
    WorldState:    "RED PILLAR at 300mm"   (confirmed by BOTH sensors)

The decision layer (state machine) only reads WorldState.
It never touches raw camera or LIDAR data directly.
This separation makes the code cleaner and easier to test.
"""

from __future__ import annotations

from dataclasses import dataclass, field


@dataclass
class WallInfo:
    """Wall distances from LIDAR.

    The robot drives in a corridor between walls.
    We need to know how far each wall is so we can stay centered.

       left wall                right wall
         |                        |
         |     <-- robot -->      |
         |                        |
         |<-- left_dist -->       |
         |       <-- right_dist -->|
         |                        |
         |<---- corridor_width --->|
    """

    left_distance: float | None = None   # mm, None if not visible
    right_distance: float | None = None  # mm
    front_distance: float | None = None  # mm

    @property
    def corridor_width(self) -> float | None:
        """Total corridor width = left + right distance."""
        if self.left_distance is None or self.right_distance is None:
            return None
        return self.left_distance + self.right_distance


@dataclass
class Pillar:
    """A confirmed pillar (camera color + LIDAR distance matched).

    WRO rules:
      - RED pillar  → robot must pass on the RIGHT side
      - GREEN pillar → robot must pass on the LEFT side

    A pillar is "confirmed" when:
      1. Camera sees a colored blob at some angle
      2. LIDAR sees a small object at the SAME angle
      3. SensorFusion matches them → creates this Pillar object

    If only the camera sees red but LIDAR sees nothing there,
    it might be a false positive (red t-shirt in the audience).
    We don't create a Pillar for unconfirmed detections.
    """

    color: str       # "red" or "green"
    angle: float     # Degrees from center (positive = right, negative = left)
    distance: float  # mm (from LIDAR measurement)

    @property
    def pass_side(self) -> str:
        """Which side should the robot pass on?

        RED pillar → pass on RIGHT (steer LEFT to avoid)
        GREEN pillar → pass on LEFT (steer RIGHT to avoid)
        """
        return "right" if self.color == "red" else "left"

    def is_blocking(self, angle_threshold: float = 30.0) -> bool:
        """Is this pillar roughly ahead of us?

        If the pillar is at angle 5 degrees → it's ahead, we must avoid it.
        If the pillar is at angle 50 degrees → it's to the side, ignore it.
        """
        return abs(self.angle) < angle_threshold


@dataclass
class WorldState:
    """Everything the robot knows about the world RIGHT NOW.

    This is the ONLY thing the decision layer (state machine) reads.
    It doesn't care HOW we got this information (camera, LIDAR, etc).
    It just reads WorldState and decides what to do.

    Example:
        world = WorldState(
            walls=WallInfo(left=400, right=350, front=2000),
            pillars=[Pillar("red", angle=12.0, distance=500)],
            corner_ahead=None,
        )

        # Decision layer reads:
        if world.has_pillars:
            pillar = world.closest_pillar
            print(f"Avoid {pillar.color} pillar, pass {pillar.pass_side}")
    """

    # Wall information (from LIDAR)
    walls: WallInfo = field(default_factory=WallInfo)

    # Confirmed pillars (from camera + LIDAR fusion)
    pillars: list[Pillar] = field(default_factory=list)

    # Corner detection (from LIDAR - front wall close)
    corner_ahead: str | None = None  # "LEFT", "RIGHT", or None

    # ── Convenience properties ──────────────────────────────

    @property
    def has_pillars(self) -> bool:
        """Are there any confirmed pillars?"""
        return len(self.pillars) > 0

    @property
    def closest_pillar(self) -> Pillar | None:
        """Get the nearest pillar (if any)."""
        if not self.pillars:
            return None
        return min(self.pillars, key=lambda p: p.distance)

    @property
    def blocking_pillar(self) -> Pillar | None:
        """Get the pillar that's directly ahead and needs avoiding."""
        blocking = [p for p in self.pillars if p.is_blocking()]
        if not blocking:
            return None
        return min(blocking, key=lambda p: p.distance)

    @property
    def is_corner_approaching(self) -> bool:
        """Is there a corner ahead?"""
        return self.corner_ahead is not None
