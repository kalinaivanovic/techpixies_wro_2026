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
    Steer proportionally to pillar distance and angle.

    Closer pillar = sharper turn. Direction based on pillar color.
    Pillar angle adds extra correction (pillar to the right → steer harder left).
    """

    def __init__(
        self,
        slow_speed: int = 35,
        steering_center: int = 90,
        max_steer_offset: int = 55,
        min_steer_offset: int = 20,
        max_distance: float = 800.0,
        angle_gain: float = 0.5,
        steering_min: int = 35,
        steering_max: int = 145,
    ):
        self.slow_speed = slow_speed
        self.steering_center = steering_center
        self.max_steer_offset = max_steer_offset
        self.min_steer_offset = min_steer_offset
        self.max_distance = max_distance
        self.angle_gain = angle_gain
        self.steering_min = steering_min
        self.steering_max = steering_max

    def compute(
        self, pillar: Pillar, world: WorldState
    ) -> Tuple[int, int]:
        # RED → pass on RIGHT → steer LEFT (steering < 90)
        # GREEN → pass on LEFT → steer RIGHT (steering > 90)
        direction = -1 if pillar.color == "red" else 1

        # Urgency from distance: 0.0 when far, 1.0 when close
        # Quadratic curve — ramps up faster at medium distance
        linear = 1.0 - min(pillar.distance / self.max_distance, 1.0)
        urgency = linear * linear
        base_offset = self.min_steer_offset + urgency * (
            self.max_steer_offset - self.min_steer_offset
        )

        # Angle correction: steer harder when pillar is on the pass side.
        # pillar.angle: positive = right, negative = left
        # RED (direction=-1): pillar right (+angle) → needs more left steer
        # GREEN (direction=+1): pillar left (-angle) → needs more right steer
        angle_correction = -direction * pillar.angle * self.angle_gain

        offset = int(base_offset + angle_correction)
        offset = max(self.min_steer_offset, min(self.max_steer_offset, offset))

        steering = self.steering_center + (direction * offset)
        steering = max(self.steering_min, min(self.steering_max, steering))

        return self.slow_speed, steering
