"""
Runtime tunable parameters with JSON persistence.

All layers share one Parameters instance. The web interface
can modify values at runtime; changes take effect on the next
sensor read cycle. Single-threaded asyncio means no locks needed.
"""

from __future__ import annotations

import json
import logging
from dataclasses import asdict, dataclass
from pathlib import Path

logger = logging.getLogger(__name__)

PARAMS_DIR = Path(__file__).parent
PARAMS_FILE = PARAMS_DIR / "params.json"  # Default / last used
PARAMS_OPEN_FILE = PARAMS_DIR / "params_open.json"
PARAMS_OBSTACLE_FILE = PARAMS_DIR / "params_obstacle.json"


@dataclass
class Parameters:
    """Runtime tunable parameters."""

    # Challenge mode: "open" (no pillars) or "obstacle" (pillars + parking)
    challenge_mode: str = "obstacle"

    # Red range 1 (low hue end: 0-10)
    red_h_min1: int = 0
    red_h_max1: int = 10
    red_s_min1: int = 100
    red_s_max1: int = 255
    red_v_min1: int = 100
    red_v_max1: int = 255

    # Red range 2 (high hue end: 160-180)
    red_h_min2: int = 160
    red_h_max2: int = 180
    red_s_min2: int = 100
    red_s_max2: int = 255
    red_v_min2: int = 100
    red_v_max2: int = 255

    # Green
    green_h_min: int = 40
    green_h_max: int = 80
    green_s_min: int = 50
    green_s_max: int = 255
    green_v_min: int = 50
    green_v_max: int = 255

    # Magenta (parking markers)
    magenta_h_min: int = 140
    magenta_h_max: int = 160
    magenta_s_min: int = 100
    magenta_s_max: int = 255
    magenta_v_min: int = 100
    magenta_v_max: int = 255

    # Orange floor line (CMYK 0,60,100,0 → approx HSV 10-25)
    orange_h_min: int = 5
    orange_h_max: int = 25
    orange_s_min: int = 100
    orange_s_max: int = 255
    orange_v_min: int = 100
    orange_v_max: int = 255

    # Blue floor line (CMYK 100,80,0,0 → approx HSV 100-130)
    blue_h_min: int = 90
    blue_h_max: int = 130
    blue_s_min: int = 50
    blue_s_max: int = 255
    blue_v_min: int = 50
    blue_v_max: int = 255

    # Floor line detection
    line_y_fraction: float = 0.6  # Only detect in bottom 40% (y > 60% of frame height)
    line_min_contour_area: int = 200  # Minimum pixels for floor line blob

    # Camera resolution (restart camera to apply changes)
    camera_width: int = 640
    camera_height: int = 480

    # Detection
    min_contour_area: int = 300

    # Wheel & encoder (for speed/distance calculation)
    wheel_diameter_mm: float = 65.0  # DFRobot FIT0003 rubber wheel
    encoder_cpr: int = 1365  # 341.2 PPR × 4 (quadrature) — DFRobot FIT0450 34:1

    # Auto mode speed control
    auto_normal_speed: int = 60  # Speed for wall following (0-100)
    auto_slow_speed: int = 35  # Speed for avoidance/corners (0-100)

    # Wall following
    wall_follow_kp: float = 0.5  # Proportional gain for wall centering
    wall_follow_steer_min: int = 30  # Max steering left (degrees)
    wall_follow_steer_max: int = 150  # Max steering right (degrees)
    pre_corner_distance: int = 1200  # mm — start slowing when front wall closer than this

    # Avoidance steering (degrees offset from center 90)
    avoid_steer_min: int = 45  # Minimum turn when pillar detected
    avoid_steer_max: int = 80  # Maximum turn when pillar very close
    blocking_angle: float = 35.0  # Degrees from center — pillar within this is "blocking"

    # Corner detection and handling
    corner_threshold: int = 500  # mm — front wall distance to enter corner state
    corner_exit_threshold: int = 1200  # mm — front distance to exit corner (hysteresis)
    corner_turn_offset: int = 25  # degrees offset from center (90 ± this)
    corner_speed: int = 35  # speed during corner turn
    corner_min_frames: int = 15  # minimum frames to stay in corner (safety net)
    corner_suppression_ticks: int = 4000  # encoder ticks — no re-entry after corner exit

    # Wall collision avoidance (reverse when about to hit wall)
    wall_collision_distance: int = 250  # mm — reverse if front wall closer than this
    outer_wall_collision_distance: int = 120  # mm — reverse if outer side wall closer than this during a turn

    # Recovery (reverse-and-retry when stuck)
    recovery_reverse_frames: int = 25  # frames to reverse (~0.5s)
    recovery_reverse_speed: int = 40  # reverse speed (0-100)
    recovery_escalate: bool = True  # True = each attempt reverses longer, False = fixed duration

    # LIDAR filtering
    lidar_min_distance: int = 60  # mm, ignore readings closer (robot body)
    lidar_min_quality: int = 10  # 0-47, minimum quality to accept
    lidar_max_distance: int = 3000  # mm, ignore readings further
    lidar_display_angle: int = 180  # ± degrees from forward
    lidar_instant: bool = False  # True = update scan per-point, False = batch per rotation

    def update(self, **kwargs):
        """Update parameters from dict (e.g., from web API)."""
        for key, value in kwargs.items():
            if hasattr(self, key):
                expected_type = type(getattr(self, key))
                try:
                    setattr(self, key, expected_type(value))
                except (TypeError, ValueError):
                    logger.warning(f"Invalid value for {key}: {value}")

    def _mode_file(self) -> Path:
        """Get the file path for the current challenge mode."""
        if self.challenge_mode == "open":
            return PARAMS_OPEN_FILE
        return PARAMS_OBSTACLE_FILE

    def save(self):
        """Persist to mode-specific JSON file and default file."""
        data = asdict(self)
        # Save to mode-specific file
        mode_file = self._mode_file()
        with open(mode_file, "w") as f:
            json.dump(data, f, indent=2)
        # Also save as default (last used)
        with open(PARAMS_FILE, "w") as f:
            json.dump(data, f, indent=2)
        logger.info(f"Parameters saved to {mode_file} and {PARAMS_FILE}")

    def load_mode(self, mode: str):
        """Switch to a different challenge mode, loading its preset."""
        mode_file = PARAMS_OPEN_FILE if mode == "open" else PARAMS_OBSTACLE_FILE
        if mode_file.exists():
            try:
                with open(mode_file) as f:
                    data = json.load(f)
                self.update(**data)
                self.challenge_mode = mode
                logger.info(f"Switched to {mode} mode from {mode_file}")
            except Exception as e:
                logger.warning(f"Failed to load {mode_file}: {e}, keeping current params")
                self.challenge_mode = mode
        else:
            # No preset for this mode yet — just switch the flag
            self.challenge_mode = mode
            logger.info(f"Switched to {mode} mode (no preset file, using current params)")

    @classmethod
    def load(cls) -> Parameters:
        """Load from JSON file, or return defaults."""
        if PARAMS_FILE.exists():
            try:
                with open(PARAMS_FILE) as f:
                    data = json.load(f)
                params = cls()
                params.update(**data)
                logger.info(f"Parameters loaded from {PARAMS_FILE}")
                return params
            except Exception as e:
                logger.warning(f"Failed to load {PARAMS_FILE}: {e}, using defaults")
        return cls()

    def to_dict(self) -> dict:
        """Convert to dict for JSON API."""
        return asdict(self)
