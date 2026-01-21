"""
State machine for race control.

Manages high-level robot states:
- IDLE: Waiting to start
- RACING: Running laps
- PARKING: Parallel parking sequence
- DONE: Race complete
"""

from enum import Enum, auto


class RobotState(Enum):
    """Robot state enumeration."""

    IDLE = auto()
    RACING = auto()
    PARKING = auto()
    DONE = auto()


class StateMachine:
    """High-level state machine for race control."""

    def __init__(self):
        """Initialize state machine."""
        self.state = RobotState.IDLE
        self.lap_count = 0
        self.target_laps = 3
        self.direction = None  # 'clockwise' or 'counterclockwise'

    def start(self):
        """Start the race."""
        self.state = RobotState.RACING
        self.lap_count = 0
        # TODO: Detect initial direction from first corner

    def update(self, perception) -> RobotState:
        """
        Update state machine based on perception.

        Args:
            perception: SensorFusion instance

        Returns:
            Current state
        """
        # TODO: Implement state transitions
        # - Count laps using corner detection
        # - Transition to PARKING after 3 laps (obstacle challenge)
        # - Transition to DONE after parking or 3 laps (open challenge)
        return self.state

    def stop(self):
        """Stop the robot."""
        self.state = RobotState.DONE
