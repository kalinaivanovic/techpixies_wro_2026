"""
State machine for robot control.

Manages high-level states and transitions based on WorldState.
Each state delegates to a swappable strategy for computing (speed, steering).
"""

import logging
from enum import Enum, auto
from typing import Optional, Tuple

from config import STEERING_CENTER
from perception import WorldState, TrackMap
from strategies import (
    WallFollowStrategy,
    ProportionalWallFollow,
    AvoidanceStrategy,
    ProportionalAvoidance,
    CornerStrategy,
    LidarCornerDetection,
    ParkingStrategy,
)

logger = logging.getLogger(__name__)


class RobotState(Enum):
    """Robot state enumeration."""

    IDLE = auto()
    WALL_FOLLOW = auto()
    AVOID_PILLAR = auto()
    CORNER = auto()
    PARKING = auto()
    DONE = auto()


class StateMachine:
    """
    High-level state machine for race control.

    States:
    - IDLE: Waiting to start
    - WALL_FOLLOW: Default driving, maintain corridor center
    - AVOID_PILLAR: Pillar detected, navigate around
    - CORNER: Corner ahead, execute turn
    - PARKING: Parallel parking sequence
    - DONE: Race complete

    Usage:
        sm = StateMachine()
        sm.start()

        # In control loop:
        speed, steering = sm.decide(world_state, track_map)

        # With custom strategies:
        sm = StateMachine(
            wall_follow=ProportionalWallFollow(kp=0.8),
            avoidance=ProportionalAvoidance(max_steer_offset=30),
        )
    """

    def __init__(
        self,
        wall_follow: WallFollowStrategy = None,
        avoidance: AvoidanceStrategy = None,
        corner: CornerStrategy = None,
        parking: ParkingStrategy = None,
        params=None,
    ):
        self.state = RobotState.IDLE
        self.lap_count = 0
        self.target_laps = 3
        self.direction: Optional[str] = None  # "CW" or "CCW"
        self.params = params  # Shared Parameters for runtime speed tuning

        # Strategies
        self.wall_follow = wall_follow or ProportionalWallFollow()
        self.avoidance = avoidance or ProportionalAvoidance()
        self.corner = corner or LidarCornerDetection()
        self.parking = parking  # None until implemented

        # For pillar avoidance state
        self._avoiding_pillar: Optional[str] = None  # Color being avoided
        self._avoid_phase = 0  # 0=approach, 1=passing, 2=returning
        self._avoid_frames = 0  # How many frames in AVOID_PILLAR
        self._min_avoid_frames = 15  # Stay in avoidance at least this many frames (~0.3s at 50Hz)

    def start(self):
        """Start the race."""
        self.state = RobotState.WALL_FOLLOW
        self.lap_count = 0
        self._avoiding_pillar = None
        logger.info("Race started")

    def stop(self):
        """Stop the robot."""
        self.state = RobotState.DONE
        logger.info("Race stopped")

    def decide(
        self,
        world: WorldState,
        track_map: TrackMap,
    ) -> Tuple[int, int]:
        """
        Decide speed and steering based on current perception.

        Args:
            world: Current WorldState from sensor fusion
            track_map: Accumulated track knowledge

        Returns:
            (speed, steering) tuple
        """
        if self.state == RobotState.IDLE:
            return 0, STEERING_CENTER

        if self.state == RobotState.DONE:
            return 0, STEERING_CENTER

        # Sync strategy speeds from runtime params
        if self.params:
            self.wall_follow.normal_speed = self.params.auto_normal_speed
            self.avoidance.slow_speed = self.params.auto_slow_speed
            self.corner.slow_speed = self.params.auto_slow_speed

        # Update direction from track map
        if self.direction is None and track_map.direction:
            self.direction = track_map.direction

        # Check state transitions
        self._check_transitions(world, track_map)

        # Execute current state via strategy
        if self.state == RobotState.WALL_FOLLOW:
            return self.wall_follow.compute(world)

        elif self.state == RobotState.AVOID_PILLAR:
            # Use blocking pillar, or closest pillar, or any visible pillar
            pillar = world.blocking_pillar or world.closest_pillar
            if pillar is None:
                # Pillar lost from view — keep steering in avoidance direction
                direction = -1 if self._avoiding_pillar == "red" else 1
                steer = self.avoidance.steering_center + (direction * self.avoidance.min_steer_offset)
                logger.info(
                    f"AVOID blind: no pillar visible, holding steer={steer}° "
                    f"(was {self._avoiding_pillar}, frame {self._avoid_frames})"
                )
                return self.avoidance.slow_speed, steer
            speed, steering = self.avoidance.compute(pillar, world)
            self._update_avoid_phase(pillar)
            return speed, steering

        elif self.state == RobotState.CORNER:
            direction = world.corner_ahead or "RIGHT"
            return self.corner.compute(direction, world)

        elif self.state == RobotState.PARKING:
            if self.parking is not None:
                return self.parking.compute(world)
            return 0, STEERING_CENTER

        return 0, STEERING_CENTER

    def _check_transitions(self, world: WorldState, track_map: TrackMap) -> None:
        """Check and execute state transitions."""

        if self.state == RobotState.WALL_FOLLOW:
            # Priority 1: Pillar detected
            if world.blocking_pillar:
                self.state = RobotState.AVOID_PILLAR
                self._avoiding_pillar = world.blocking_pillar.color
                self._avoid_phase = 0
                self._avoid_frames = 0
                p = world.blocking_pillar
                logger.info(
                    f"Transition: WALL_FOLLOW -> AVOID_PILLAR "
                    f"({p.color} dist={p.distance:.0f}mm angle={p.angle:.1f}°)"
                )

            # Priority 2: Corner detected
            elif world.is_corner_approaching:
                self.state = RobotState.CORNER
                logger.info(f"Transition: WALL_FOLLOW -> CORNER ({world.corner_ahead})")

            # Priority 3: Parking (lap 3 + parking visible)
            elif self.lap_count >= 3 and world.is_parking_visible:
                self.state = RobotState.PARKING
                if self.parking is not None:
                    self.parking.reset()
                logger.info("Transition: WALL_FOLLOW -> PARKING")

        elif self.state == RobotState.AVOID_PILLAR:
            self._avoid_frames += 1
            # Stay in avoidance for minimum frames to actually complete the maneuver
            if self._avoid_frames < self._min_avoid_frames:
                return
            # Return to wall follow when pillar cleared
            if not world.blocking_pillar or self._avoid_phase >= 2:
                self.state = RobotState.WALL_FOLLOW
                self._avoiding_pillar = None
                logger.info(f"Transition: AVOID_PILLAR -> WALL_FOLLOW (after {self._avoid_frames} frames)")

        elif self.state == RobotState.CORNER:
            # Return to wall follow when corner cleared
            if not world.is_corner_approaching:
                self.state = RobotState.WALL_FOLLOW
                # Check if we completed a lap (counted by 4 corners)
                if track_map.corner_count > 0 and track_map.corner_count % 4 == 0:
                    self.lap_count = track_map.corner_count // 4
                    logger.info(f"Lap {self.lap_count} complete")
                    if self.lap_count >= self.target_laps:
                        self.state = RobotState.DONE
                        logger.info("Race complete!")
                logger.info("Transition: CORNER -> WALL_FOLLOW")

        elif self.state == RobotState.PARKING:
            if self.parking is not None and self.parking.is_complete():
                self.state = RobotState.DONE
                logger.info("Transition: PARKING -> DONE")

    def _update_avoid_phase(self, pillar) -> None:
        """Track pillar avoidance progress."""
        if pillar.distance < 300:
            self._avoid_phase = 1  # Passing
        elif self._avoid_phase == 1 and pillar.distance > 400:
            self._avoid_phase = 2  # Done
