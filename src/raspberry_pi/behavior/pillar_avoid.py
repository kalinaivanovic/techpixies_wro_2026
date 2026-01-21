"""
Pillar avoidance behavior.

Avoids traffic sign pillars according to WRO rules:
- Red pillar: pass on RIGHT
- Green pillar: pass on LEFT
"""


class PillarAvoider:
    """Pillar avoidance behavior."""

    def __init__(self):
        """Initialize pillar avoider."""
        pass

    def compute(self, pillar: dict | None, wall_distances: dict) -> tuple[float, float] | None:
        """
        Compute avoidance maneuver if pillar detected.

        Args:
            pillar: Pillar info dict with 'color', 'angle', 'distance' or None
            wall_distances: Current wall distances for safety

        Returns:
            Tuple of (speed, steering_angle) or None if no avoidance needed
        """
        if pillar is None:
            return None

        # TODO: Implement avoidance logic
        # Red -> steer right
        # Green -> steer left
        return None
