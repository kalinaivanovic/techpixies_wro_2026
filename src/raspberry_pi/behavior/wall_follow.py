"""
Wall following behavior.

Keeps the robot centered in the corridor by maintaining
equal distance from both walls.
"""


class WallFollower:
    """Wall following behavior for corridor navigation."""

    def __init__(self, target_distance: float = 300):
        """
        Initialize wall follower.

        Args:
            target_distance: Target distance from wall in mm
        """
        self.target_distance = target_distance

    def compute(self, wall_distances: dict) -> tuple[float, float]:
        """
        Compute speed and steering based on wall distances.

        Args:
            wall_distances: Dict with 'front', 'left', 'right' in mm

        Returns:
            Tuple of (speed, steering_angle)
        """
        # TODO: Implement wall following logic
        return (0, 0)
