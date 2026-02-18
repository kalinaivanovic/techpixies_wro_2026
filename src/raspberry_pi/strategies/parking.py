"""
Parking strategies.

Handles the parallel parking maneuver at the end of lap 3.
"""

from abc import ABC, abstractmethod
from typing import Tuple

from perception.world_state import WorldState


class ParkingStrategy(ABC):
    """Base class for parking maneuver algorithms."""

    @abstractmethod
    def compute(self, world: WorldState) -> Tuple[int, int]:
        """
        Compute speed and steering for parking maneuver.

        Args:
            world: Current world state.

        Returns:
            (speed, steering) tuple.
        """
        ...

    @abstractmethod
    def is_complete(self) -> bool:
        """Check if parking maneuver is finished."""
        ...

    @abstractmethod
    def reset(self) -> None:
        """Reset parking state for a new attempt."""
        ...
