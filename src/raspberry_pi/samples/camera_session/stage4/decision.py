"""
Decision layer - What should the robot DO?

Stage 4: The top of the pipeline.

The full chain:
  Camera → blobs → Fusion → WorldState → Decision → (speed, steering)

The decision layer reads WorldState and outputs a simple command:
  speed (how fast) and steering (which direction).

It uses a STATE MACHINE with these states:

  ┌─────────────┐
  │ WALL_FOLLOW  │  ← default: drive forward, stay centered
  └──────┬───────┘
         │ pillar detected?
         ▼
  ┌─────────────┐
  │ AVOID_PILLAR │  ← steer around the pillar
  └──────┬───────┘
         │ pillar cleared?
         ▼
  back to WALL_FOLLOW

In this demo we only show WALL_FOLLOW and AVOID_PILLAR.
The real robot also has CORNER and PARKING states.

Key concept: The state machine doesn't know about cameras or LIDAR.
It only knows about WorldState. This means:
  - We can TEST the decision logic with fake WorldState objects
  - We can SWAP sensors without changing the decision code
  - Each layer has a single, clear responsibility
"""

from enum import Enum, auto
from typing import Optional, Tuple

from world_state import WorldState, Pillar


# Steering center = 90 degrees (straight ahead for our servo)
STEERING_CENTER = 90


class RobotState(Enum):
    """What is the robot currently doing?

    Simple state machine:
      WALL_FOLLOW → AVOID_PILLAR → WALL_FOLLOW → ...

    Real robot has more states (CORNER, PARKING, DONE),
    but these two show the core concept.
    """

    WALL_FOLLOW = auto()    # Default: drive forward, stay centered
    AVOID_PILLAR = auto()   # Pillar ahead: steer around it


class Decision:
    """
    Decides what the robot should do based on WorldState.

    Usage:
        decision = Decision()
        speed, steering = decision.decide(world_state)
        # speed: -100 to +100 (motor power)
        # steering: 0 to 180 (servo angle, 90 = straight)

    The decide() method:
      1. Checks if we need to change state (transition)
      2. Runs the current state's logic
      3. Returns (speed, steering)

    Example flow:
      Frame 1: WALL_FOLLOW, no pillars → speed=40, steer=90 (straight)
      Frame 2: WALL_FOLLOW, red pillar at 500mm → transition to AVOID_PILLAR
      Frame 3: AVOID_PILLAR, red pillar → speed=30, steer=70 (turn left)
      Frame 4: AVOID_PILLAR, pillar passed → transition to WALL_FOLLOW
      Frame 5: WALL_FOLLOW, clear ahead → speed=40, steer=90 (straight)
    """

    def __init__(self):
        self.state = RobotState.WALL_FOLLOW
        self._avoiding_color: Optional[str] = None

    def decide(self, world: WorldState) -> Tuple[int, int]:
        """Main decision function.

        Args:
            world: Current WorldState from sensor fusion

        Returns:
            (speed, steering) tuple:
              speed: -100 to +100
              steering: 0 to 180 (90 = straight)
        """
        # Check if we need to change state
        self._check_transitions(world)

        # Execute current state
        if self.state == RobotState.WALL_FOLLOW:
            return self._wall_follow(world)
        elif self.state == RobotState.AVOID_PILLAR:
            return self._avoid_pillar(world)

        return 0, STEERING_CENTER

    def _check_transitions(self, world: WorldState) -> None:
        """Should we change state?

        State transitions are based on WorldState:
          - WALL_FOLLOW + blocking pillar → AVOID_PILLAR
          - AVOID_PILLAR + no blocking pillar → WALL_FOLLOW
        """
        if self.state == RobotState.WALL_FOLLOW:
            if world.blocking_pillar:
                self.state = RobotState.AVOID_PILLAR
                self._avoiding_color = world.blocking_pillar.color
                print(f"→ AVOID_PILLAR ({self._avoiding_color})")

        elif self.state == RobotState.AVOID_PILLAR:
            if not world.blocking_pillar:
                self.state = RobotState.WALL_FOLLOW
                self._avoiding_color = None
                print("→ WALL_FOLLOW")

    def _wall_follow(self, world: WorldState) -> Tuple[int, int]:
        """Default behavior: drive forward, stay centered in corridor.

        Uses proportional control:
          error = right_wall - left_wall
          If right wall is farther → we're too far left → steer right
          If left wall is farther → we're too far right → steer left

        This is a simple proportional (P) controller.
        """
        speed = 40
        steering = STEERING_CENTER  # Start with straight

        walls = world.walls
        if walls.left_distance and walls.right_distance:
            # Error: positive = we're left of center
            error = walls.right_distance - walls.left_distance

            # Proportional gain (tune this!)
            kp = 0.05

            # Steering correction
            steering = STEERING_CENTER + int(error * kp)

            # Clamp to valid range
            steering = max(60, min(120, steering))

        return speed, steering

    def _avoid_pillar(self, world: WorldState) -> Tuple[int, int]:
        """Avoid a pillar based on its color.

        WRO rules:
          RED pillar   → pass on RIGHT → steer LEFT (decrease steering)
          GREEN pillar → pass on LEFT  → steer RIGHT (increase steering)

        The amount we steer depends on distance:
          - Far away (>800mm): gentle steer
          - Close (<400mm): sharp steer
        """
        pillar = world.blocking_pillar
        if pillar is None:
            return self._wall_follow(world)

        speed = 30  # Slow down for safety

        # Which way to steer?
        if pillar.color == "red":
            # Pass on RIGHT → steer LEFT
            steer_direction = -1
        else:
            # Pass on LEFT → steer RIGHT
            steer_direction = +1

        # How much to steer? More if pillar is close.
        if pillar.distance < 400:
            steer_amount = 30  # Sharp turn
        elif pillar.distance < 800:
            steer_amount = 20  # Medium turn
        else:
            steer_amount = 10  # Gentle turn

        steering = STEERING_CENTER + (steer_direction * steer_amount)
        steering = max(45, min(135, steering))

        return speed, steering
