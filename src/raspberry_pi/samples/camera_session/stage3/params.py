"""
Parameters - Runtime tunable values.

Stage 3: Instead of hardcoded HSV values in camera.py, we put them
in a shared Parameters object. This way:
  1. All settings are in one place (not scattered across files)
  2. We can change them at runtime (no restart needed!)
  3. We can save them to disk and load on next startup

This is a @dataclass:
  - We list the fields with types and defaults
  - Python auto-generates __init__, __repr__, __eq__
  - So Parameters() gives us an object with all defaults set

Why not just use a dict?
  - Typos: params["gren_h_min"] silently creates a wrong key
  - No autocomplete in IDE
  - No type checking
  - With dataclass: params.gren_h_min -> AttributeError (caught immediately)
"""

import json
from dataclasses import dataclass, asdict
from pathlib import Path

# File to persist parameters (same directory as this script)
PARAMS_FILE = Path(__file__).parent / "params.json"


@dataclass
class Parameters:
    """All tunable settings in one place."""

    # ── HSV color ranges ────────────────────────────────────────
    #
    # OpenCV HSV: H = 0-180, S = 0-255, V = 0-255
    #
    # Hue scale:
    # Red     Orange    Yellow    Green     Cyan      Blue      Magenta   Red
    #  0        15        30        60        90       120        150      180
    #
    # Red wraps around both ends, so we need TWO ranges.

    # Red range 1 (low hue: 0-10)
    red_h_min: int = 0
    red_h_max: int = 10
    red_s_min: int = 100
    red_s_max: int = 255
    red_v_min: int = 100
    red_v_max: int = 255

    # Red range 2 (high hue: 160-180)
    red_h_min2: int = 160
    red_h_max2: int = 180
    red_s_min2: int = 100
    red_s_max2: int = 255
    red_v_min2: int = 100
    red_v_max2: int = 255

    # Green (single range, sits in middle of hue spectrum)
    green_h_min: int = 40
    green_h_max: int = 80
    green_s_min: int = 50
    green_s_max: int = 255
    green_v_min: int = 50
    green_v_max: int = 255

    # Magenta / pink (parking markers)
    magenta_h_min: int = 140
    magenta_h_max: int = 160
    magenta_s_min: int = 100
    magenta_s_max: int = 255
    magenta_v_min: int = 100
    magenta_v_max: int = 255

    # ── Detection settings ──────────────────────────────────────

    # Minimum blob area in pixels (filter out small noise)
    min_area: int = 300

    # ── Methods ─────────────────────────────────────────────────

    def update(self, **kwargs):
        """Update parameters from a dict.

        Example:
            params.update(red_h_min=5, green_h_max=90)

        Only updates fields that actually exist on the dataclass.
        Ignores unknown keys (safe to call with user input).
        """
        for key, value in kwargs.items():
            if hasattr(self, key):
                # Cast to the correct type (int stays int, etc.)
                expected_type = type(getattr(self, key))
                setattr(self, key, expected_type(value))

    def save(self):
        """Save current parameters to params.json."""
        with open(PARAMS_FILE, "w") as f:
            json.dump(asdict(self), f, indent=2)

    @classmethod
    def load(cls) -> "Parameters":
        """Load from params.json, or return defaults if file doesn't exist."""
        if PARAMS_FILE.exists():
            with open(PARAMS_FILE) as f:
                data = json.load(f)
            params = cls()
            params.update(**data)
            return params
        return cls()
