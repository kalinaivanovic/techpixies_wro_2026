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

PARAMS_FILE = Path(__file__).parent / "params.json"


@dataclass
class Parameters:
    """Runtime tunable parameters."""

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

    # Detection
    min_contour_area: int = 300

    def update(self, **kwargs):
        """Update parameters from dict (e.g., from web API)."""
        for key, value in kwargs.items():
            if hasattr(self, key):
                expected_type = type(getattr(self, key))
                try:
                    setattr(self, key, expected_type(value))
                except (TypeError, ValueError):
                    logger.warning(f"Invalid value for {key}: {value}")

    def save(self):
        """Persist to JSON file."""
        with open(PARAMS_FILE, "w") as f:
            json.dump(asdict(self), f, indent=2)
        logger.info(f"Parameters saved to {PARAMS_FILE}")

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
