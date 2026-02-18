"""
Pillar avoidance strategies.

Takes a detected pillar and world state, returns (speed, steering)
to navigate around the pillar on the correct side.

WRO rules:
- RED pillar → pass on RIGHT (steer left)
- GREEN pillar → pass on LEFT (steer right)
"""

from abc import ABC, abstractmethod
from typing import Tuple

from perception.world_state import Pillar, WorldState


class AvoidanceStrategy(ABC):
    """Base class for pillar avoidance algorithms."""

    @abstractmethod
    def compute(
        self, pillar: Pillar, world: WorldState
    ) -> Tuple[int, int]:
        """
        Compute speed and steering to avoid a pillar.

        Args:
            pillar: The pillar to avoid.
            world: Current world state.

        Returns:
            (speed, steering) tuple.
        """
        ...


class ProportionalAvoidance(AvoidanceStrategy):
    """
    Steer proportionally to pillar distance.

    Closer pillar = sharper turn. Direction based on pillar color.
    """

    def __init__(
        self,
        slow_speed: int = 35,
        steering_center: int = 90,
        max_steer_offset: int = 25,
        max_distance: float = 800.0,
        steering_min: int = 60,
        steering_max: int = 120,
    ):
        self.slow_speed = slow_speed
        self.steering_center = steering_center
        self.max_steer_offset = max_steer_offset
        self.max_distance = max_distance
        self.steering_min = steering_min
        self.steering_max = steering_max

    def compute(
        self, pillar: Pillar, world: WorldState
    ) -> Tuple[int, int]:
        # RED → steer left (-1) to pass on right
        # GREEN → steer right (+1) to pass on left
        direction = -1 if pillar.color == "red" else 1

        # Urgency: 0.0 when far, 1.0 when close
        urgency = 1.0 - min(pillar.distance / self.max_distance, 1.0)
        offset = int(urgency * self.max_steer_offset)

        steering = self.steering_center + (direction * offset)
        steering = max(self.steering_min, min(self.steering_max, steering))

        return self.slow_speed, steering
