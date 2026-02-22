"""
Wall following strategies - Maintain position in corridor.

Takes WorldState, returns (speed, steering).
"""

from __future__ import annotations

from abc import ABC, abstractmethod

from perception.world_state import WorldState


class WallFollowStrategy(ABC):
    """Base class for wall following algorithms."""

    @abstractmethod
    def compute(self, world: WorldState) -> tuple[int, int]:
        """
        Compute speed and steering to follow walls.

        Args:
            world: Current world state with wall distances.

        Returns:
            (speed, steering) where steering center is 90.
        """
        ...


class ProportionalWallFollow(WallFollowStrategy):
    """
    Simple proportional control to stay centered in corridor.

    Measures distance to both walls, calculates error from center,
    applies proportional gain to correct steering.
    """

    def __init__(
        self,
        kp: float = 0.5,
        normal_speed: int = 60,
        steering_center: int = 90,
        min_clearance: int = 150,
        steering_min: int = 60,
        steering_max: int = 120,
    ):
        self.kp = kp
        self.normal_speed = normal_speed
        self.steering_center = steering_center
        self.min_clearance = min_clearance
        self.steering_min = steering_min
        self.steering_max = steering_max

    def compute(self, world: WorldState) -> tuple[int, int]:
        speed = self.normal_speed
        left = world.walls.left_distance
        right = world.walls.right_distance

        if left is None and right is None:
            return speed, self.steering_center

        if left is None:
            # Only right wall visible
            error = right - self.min_clearance
        elif right is None:
            # Only left wall visible
            error = self.min_clearance - left
        else:
            # Both walls visible — stay centered
            corridor = left + right
            target = corridor / 2
            target = max(
                self.min_clearance,
                min(target, corridor - self.min_clearance),
            )
            error = right - target

        steering = self.steering_center + int(self.kp * error)
        steering = max(self.steering_min, min(self.steering_max, steering))

        return speed, steering
