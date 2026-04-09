"""
State machine for robot control.

Manages high-level states and transitions based on WorldState.
Each state delegates to a swappable strategy for computing (speed, steering).
"""

from __future__ import annotations

import logging
from enum import Enum, auto
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
    RECOVERY = auto()
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

    def __init__(self, wall_follow: WallFollowStrategy = None, avoidance: AvoidanceStrategy = None, corner: CornerStrategy = None, parking: ParkingStrategy = None, params=None):
        self.state = RobotState.IDLE
        self.lap_count = 0
        self.corner_count = 0  # Counts real physical corners
        self._last_corner_entry_encoder = -10000  # Encoder at last counted corner entry
        self._wall_follow_start_encoder = 0  # Encoder when WALL_FOLLOW started
        self.target_laps = 3
        self._finish_encoder = None  # Encoder when laps completed, coast before stopping
        self._finish_coast_ticks = 5000  # How far to coast after laps done
        self.direction: str | None = None  # "CW" or "CCW"
        self.params = params  # Shared Parameters for runtime speed tuning

        # Strategies
        self.wall_follow = wall_follow or ProportionalWallFollow()
        self.avoidance = avoidance or ProportionalAvoidance()
        self.corner = corner or LidarCornerDetection()
        self.parking = parking  # None until implemented

        # For pillar avoidance state
        self._avoiding_pillar: str | None = None  # Color being avoided
        self._avoid_phase = 0  # 0=approach, 1=passing, 2=returning
        self._avoid_frames = 0  # How many frames in AVOID_PILLAR
        self._min_avoid_frames = 25  # Stay in avoidance at least this many frames (~0.5s at 50Hz)

        # Clearance: pillar must be this far to the side OR this far away to be "cleared"
        # Robot is 200mm wide — at 300mm distance, half-width is arctan(100/300)=18°
        # plus pillar half-width 25mm, plus margin. Need ~65° to be safe at close range.
        self._clear_angle = 65.0  # degrees — must be well past 200mm robot body
        self._clear_distance = 600.0  # mm — or far enough to not matter

        # For corner state
        self._corner_frames = 0
        self._min_corner_frames = 15  # Safety minimum frames in corner
        self._corner_exit_threshold = 1200  # mm — front must be this clear to exit
        self._corner_direction: str | None = None  # Remember turn direction
        self._corner_min_front: float = 9999  # Track minimum front distance during turn

        # For recovery state (reverse-and-retry)
        self._recovery_frames = 0
        self._recovery_reverse_frames = 25  # How long to reverse
        self._recovery_reverse_speed = 40
        self._recovery_escalate = True  # Escalate reverse duration per attempt
        self._recovery_attempts = 0  # Count retries for this corner

        # Corner suppression after pillar avoidance
        self._corner_suppressed_frames = 0  # Countdown: skip corner detection

        # Recovery context: what state triggered recovery
        self._recovery_return_state = RobotState.CORNER  # Where to go after recovery

    def start(self):
        """Start the race."""
        self.state = RobotState.WALL_FOLLOW
        self.lap_count = 0
        self.corner_count = 0
        self._last_corner_entry_encoder = -10000
        self._wall_follow_start_encoder = 0
        self._finish_encoder = None
        self._avoiding_pillar = None
        logger.info("Race started")

    def stop(self):
        """Stop the robot."""
        self.state = RobotState.DONE
        logger.info("Race stopped")

    def decide(self, world: WorldState, track_map: TrackMap) -> tuple[int, int]:
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

        # Read parameters each time when called. Ensures that changed parameters are loaded.
        if self.params:
            self.wall_follow.normal_speed = self.params.auto_normal_speed
            self.wall_follow.slow_speed = self.params.auto_slow_speed
            self.wall_follow.kp = self.params.wall_follow_kp
            self.wall_follow.steering_min = self.params.wall_follow_steer_min
            self.wall_follow.steering_max = self.params.wall_follow_steer_max
            self.wall_follow.pre_corner_distance = self.params.pre_corner_distance
            self.avoidance.slow_speed = self.params.auto_slow_speed
            self.avoidance.min_steer_offset = self.params.avoid_steer_min
            self.avoidance.max_steer_offset = self.params.avoid_steer_max
            self.corner.slow_speed = self.params.corner_speed
            self.corner.turn_offset = self.params.corner_turn_offset
            self.corner.threshold = self.params.corner_threshold
            self._corner_exit_threshold = self.params.corner_exit_threshold
            self._min_corner_frames = self.params.corner_min_frames
            self._recovery_reverse_frames = self.params.recovery_reverse_frames
            self._recovery_reverse_speed = self.params.recovery_reverse_speed
            self._recovery_escalate = self.params.recovery_escalate

        # Update direction from track map
        if self.direction is None and track_map.direction:
            self.direction = track_map.direction

        # Check state transitions
        blocking_angle = self.params.blocking_angle if self.params else 35.0
        self._check_transitions(world, track_map, blocking_angle)

        # Execute current state via strategy
        if self.state == RobotState.WALL_FOLLOW:
            return self.wall_follow.compute(world)

        elif self.state == RobotState.AVOID_PILLAR:
            # ONLY use the pillar we're currently avoiding (same color).
            # If we switch to a different-color pillar mid-maneuver,
            # steering reverses and we hit the one we were passing.
            pillar = self._find_avoiding_pillar(world)
            if pillar is None:
                # Our pillar lost from view — keep steering hard in same direction
                direction = 1 if self._avoiding_pillar == "red" else -1
                steer = self.avoidance.steering_center + (direction * self.avoidance.max_steer_offset)
                self._log_count_blind = getattr(self, '_log_count_blind', 0) + 1
                if self._log_count_blind % 10 == 1:
                    logger.info(
                        f"AVOID blind: {self._avoiding_pillar} lost, holding steer={steer}° "
                        f"(frame {self._avoid_frames})"
                    )
                return self.avoidance.slow_speed, steer
            self._log_count_blind = 0
            speed, steering = self.avoidance.compute(pillar, world)
            self._update_avoid_phase(pillar)
            return speed, steering

        elif self.state == RobotState.CORNER:
            # Direction is locked at entry. Set track direction from first successful corner.
            if self.direction is None and self._corner_direction:
                self.direction = "CW" if self._corner_direction == "RIGHT" else "CCW"

            direction = self._corner_direction or "RIGHT"
            speed, steering = self.corner.compute(direction, world)
            if self._corner_frames % 15 == 0:
                front = world.walls.front_distance
                logger.info(
                    f"CORNER: dir={direction} steer={steering}° speed={speed} "
                    f"front={front:.0f}mm frame={self._corner_frames}"
                    if front else
                    f"CORNER: dir={direction} steer={steering}° speed={speed} "
                    f"front=None frame={self._corner_frames}"
                )
            return speed, steering

        elif self.state == RobotState.RECOVERY:
            # Reverse with OPPOSITE steering to back away from corner wall.
            # If corner is RIGHT: reverse + steer LEFT → rear swings right,
            # front pulls away from the ahead wall, creating room for next attempt.
            if self._corner_direction == "LEFT":
                steering = STEERING_CENTER + self.corner.turn_offset  # steer RIGHT while reversing
            else:
                steering = STEERING_CENTER - self.corner.turn_offset  # steer LEFT while reversing
            return -self._recovery_reverse_speed, steering

        elif self.state == RobotState.PARKING:
            if self.parking is not None:
                return self.parking.compute(world)
            return 0, STEERING_CENTER

        return 0, STEERING_CENTER

    def _check_transitions(self, world: WorldState, track_map: TrackMap, blocking_angle: float) -> None:
        """Check and execute state transitions."""

        if self.state == RobotState.WALL_FOLLOW:
            # Corner suppression: encoder-based after corner exit, frame-based after pillar
            if self._corner_suppressed_frames > 0:
                self._corner_suppressed_frames -= 1
            encoder = world.encoder_pos
            # Corner is a real new corner only if robot traveled enough in WALL_FOLLOW
            wall_follow_distance = abs(encoder - self._wall_follow_start_encoder)
            suppression_ticks = self.params.corner_suppression_ticks if self.params else 4000
            corner_suppressed = (wall_follow_distance < suppression_ticks
                                 or self._corner_suppressed_frames > 0)

            # Priority 1: Pillar detected (obstacle mode only)
            is_open = self.params and self.params.challenge_mode == "open"
            p = None if is_open else world.blocking_pillar(blocking_angle)
            if p and self.params and p.distance > self.params.avoid_trigger_distance:
                p = None  # Too far — wait until closer
            if p:
                self.state = RobotState.AVOID_PILLAR
                self._avoiding_pillar = p.color
                self._avoid_phase = 0
                self._avoid_frames = 0
                logger.info(
                    f"Transition: WALL_FOLLOW -> AVOID_PILLAR "
                    f"({p.color} dist={p.distance:.0f}mm angle={p.angle:.1f}°)"
                )

            # Priority 2: Corner detected
            # Suppressed after recent corner exit (encoder-based) or near pillars
            elif (not corner_suppressed
                  and (is_open or not self._has_close_pillar(world, blocking_angle))
                  and world.is_corner_approaching):
                self.state = RobotState.CORNER
                self._corner_frames = 0
                self._recovery_attempts = 0
                self._corner_min_front = 9999
                # Count this as a real corner (suppression already filtered false ones)
                self.corner_count += 1
                self._last_corner_entry_encoder = encoder
                logger.info(f"Corner #{self.corner_count} at encoder {encoder} (wall_follow_dist={wall_follow_distance})")
                # Use track direction to determine corner direction (more reliable)
                # CW track → all corners are RIGHT, CCW → all LEFT
                if self.direction == "CW":
                    self._corner_direction = "RIGHT"
                elif self.direction == "CCW":
                    self._corner_direction = "LEFT"
                else:
                    # Direction not yet known — use detector's guess
                    self._corner_direction = world.corner_ahead
                logger.info(
                    f"Transition: WALL_FOLLOW -> CORNER "
                    f"({self._corner_direction}, track={self.direction}, detector={world.corner_ahead})"
                )

            # Priority 3: Parking (obstacle mode only, lap 3 + parking visible)
            elif not is_open and self.lap_count >= 3 and world.is_parking_visible:
                self.state = RobotState.PARKING
                if self.parking is not None:
                    self.parking.reset()
                logger.info("Transition: WALL_FOLLOW -> PARKING")

            # Check if line counting detected a new lap (even between corners)
            line_laps = track_map.line_lap_count
            if line_laps > self.lap_count:
                self.lap_count = line_laps
                logger.info(f"Lap {self.lap_count} complete (from line detection, blue={track_map.blue_count})")
                if self.lap_count >= self.target_laps and self._finish_encoder is None:
                    self._finish_encoder = encoder
                    logger.info(f"Laps complete at encoder {encoder}, coasting {self._finish_coast_ticks} ticks...")

            # Coast after laps complete, then stop
            if self._finish_encoder is not None:
                if abs(encoder - self._finish_encoder) >= self._finish_coast_ticks:
                    self.state = RobotState.DONE
                    logger.info("Race complete! Stopped after coasting.")

        elif self.state == RobotState.AVOID_PILLAR:
            self._avoid_frames += 1

            # Check if robot is about to hit wall while avoiding pillar
            front = world.walls.front_distance
            wall_collision_dist = self.params.wall_collision_distance if self.params else 350
            if front is not None and front < wall_collision_dist and self._avoid_frames > 10:
                self.state = RobotState.RECOVERY
                self._recovery_frames = 0
                self._recovery_attempts += 1
                self._recovery_return_state = RobotState.AVOID_PILLAR
                # Set direction to SAME as avoidance steering, so RECOVERY inverts it
                # RED → avoidance steers RIGHT → set RIGHT → RECOVERY steers LEFT
                # GREEN → avoidance steers LEFT → set LEFT → RECOVERY steers RIGHT
                if self._avoiding_pillar == "red":
                    self._corner_direction = "RIGHT"
                else:
                    self._corner_direction = "LEFT"
                logger.info(
                    f"Transition: AVOID_PILLAR -> RECOVERY "
                    f"(wall at {front:.0f}mm, attempt {self._recovery_attempts})"
                )
                return

            # Stay in avoidance for minimum frames to actually complete the maneuver
            if self._avoid_frames < self._min_avoid_frames:
                return
            # Return to wall follow when pillar actually cleared
            if self._is_pillar_cleared(world):
                logger.info(f"Transition: AVOID_PILLAR -> WALL_FOLLOW (after {self._avoid_frames} frames)")
                self.state = RobotState.WALL_FOLLOW
                self._avoiding_pillar = None
                self._wall_follow_start_encoder = world.encoder_pos
                self._corner_suppressed_frames = (
                    self.params.post_pillar_corner_suppression if self.params else 30
                )

        elif self.state == RobotState.CORNER:
            self._corner_frames += 1
            # Pillar overrides corner (obstacle mode only)
            is_open = self.params and self.params.challenge_mode == "open"
            p = None if is_open else world.blocking_pillar(blocking_angle)
            if p and self.params and p.distance > self.params.avoid_trigger_distance:
                p = None  # Too far — stay in corner
            if p:
                self.state = RobotState.AVOID_PILLAR
                self._avoiding_pillar = p.color
                self._avoid_phase = 0
                self._avoid_frames = 0
                logger.info(
                    f"Transition: CORNER -> AVOID_PILLAR "
                    f"({p.color} dist={p.distance:.0f}mm angle={p.angle:.1f}°)"
                )
            # Must stay minimum frames (safety net against LIDAR noise)
            elif self._corner_frames < self._min_corner_frames:
                return
            # Hysteresis: exit when front is clearly open (straight path ahead)
            elif self._is_corner_cleared(world):
                self.state = RobotState.WALL_FOLLOW
                self._recovery_attempts = 0
                self._wall_follow_start_encoder = world.encoder_pos  # Track WALL_FOLLOW distance
                logger.info(
                    f"Transition: CORNER -> WALL_FOLLOW "
                    f"(corners={self.corner_count})"
                )
            # Stuck against wall? Trigger recovery based on front distance
            else:
                front = world.walls.front_distance
                wall_collision_dist = self.params.wall_collision_distance if self.params else 250
                if front is not None and front < wall_collision_dist:
                    self.state = RobotState.RECOVERY
                    self._recovery_frames = 0
                    self._recovery_attempts += 1
                    self._recovery_return_state = RobotState.CORNER
                    logger.info(
                        f"Transition: CORNER -> RECOVERY "
                        f"(wall at {front:.0f}mm, attempt {self._recovery_attempts})"
                    )

        elif self.state == RobotState.RECOVERY:
            self._recovery_frames += 1
            # Fixed or escalating reverse duration
            if self._recovery_escalate:
                reverse_needed = self._recovery_reverse_frames * min(self._recovery_attempts, 3)
            else:
                reverse_needed = self._recovery_reverse_frames
            if self._recovery_frames >= reverse_needed:
                # Done reversing — return to the state that triggered recovery
                return_to = self._recovery_return_state
                if return_to == RobotState.AVOID_PILLAR:
                    self.state = RobotState.WALL_FOLLOW  # Re-detect pillar fresh
                    self._avoid_frames = 0
                    self._wall_follow_start_encoder = world.encoder_pos
                elif return_to == RobotState.WALL_FOLLOW:
                    self.state = RobotState.WALL_FOLLOW  # Resume wall following
                    self._wall_follow_start_encoder = world.encoder_pos
                else:
                    self.state = RobotState.CORNER
                    self._corner_frames = 0
                logger.info(
                    f"Transition: RECOVERY -> {self.state.name} "
                    f"(reversed {self._recovery_frames} frames, attempt {self._recovery_attempts})"
                )

        elif self.state == RobotState.PARKING:
            if self.parking is not None and self.parking.is_complete():
                self.state = RobotState.DONE
                logger.info("Transition: PARKING -> DONE")

    def _is_corner_cleared(self, world: WorldState) -> bool:
        """Check if the robot has completed the corner turn.

        Original hysteresis: exit when front > corner_exit_threshold.
        """
        front = world.walls.front_distance
        if front is None:
            return False
        return front > self._corner_exit_threshold

    def _has_close_pillar(self, world: WorldState, blocking_angle: float) -> bool:
        """Check if there's a blocking pillar within the avoidance trigger distance."""
        p = world.blocking_pillar(blocking_angle)
        if p is None:
            return False
        if self.params and p.distance > self.params.avoid_trigger_distance:
            return False
        return True

    def _find_avoiding_pillar(self, world: WorldState):
        """Find the pillar we're currently avoiding (by color).

        Returns the closest pillar of the same color we started avoiding,
        or None if it's no longer visible.
        """
        matches = [p for p in world.pillars if p.color == self._avoiding_pillar]
        if not matches:
            return None
        return min(matches, key=lambda p: p.distance)

    def _is_pillar_cleared(self, world: WorldState) -> bool:
        """Check if the pillar we're avoiding is safely past the robot body.

        The pillar is "cleared" only when it's BOTH off to the side AND
        either far away or no longer visible. Requiring both prevents
        premature exit when the robot rotates fast and the angle changes
        even though the pillar is still physically in front.
        """
        # Find the pillar we're avoiding by color
        our_pillar = None
        for p in world.pillars:
            if p.color == self._avoiding_pillar:
                our_pillar = p
                break

        if our_pillar is None:
            # Pillar not visible — only clear if we've been avoiding long enough
            return self._avoid_frames > self._min_avoid_frames * 2

        # Pillar still visible — clear only if both side AND far conditions met
        # Side: well off to one side (large angle from center)
        # Far: distance is increasing past clear distance
        side_cleared = abs(our_pillar.angle) > self._clear_angle
        far_cleared = our_pillar.distance > self._clear_distance

        return side_cleared and far_cleared

    def _update_avoid_phase(self, pillar) -> None:
        """Track pillar avoidance progress."""
        if pillar.distance < 300:
            self._avoid_phase = 1  # Passing
        elif self._avoid_phase == 1 and pillar.distance > 400:
            self._avoid_phase = 2  # Done
