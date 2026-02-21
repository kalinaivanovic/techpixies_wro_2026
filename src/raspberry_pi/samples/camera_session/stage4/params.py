"""
Parameters - Same as Stage 3.

This file is identical to stage3/params.py.
We include it here so Stage 4 can run independently.
"""

import json
from dataclasses import dataclass, asdict
from pathlib import Path

PARAMS_FILE = Path(__file__).parent / "params.json"


@dataclass
class Parameters:
    """All tunable settings in one place."""

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

    # Green
    green_h_min: int = 40
    green_h_max: int = 80
    green_s_min: int = 50
    green_s_max: int = 255
    green_v_min: int = 50
    green_v_max: int = 255

    # Magenta
    magenta_h_min: int = 140
    magenta_h_max: int = 160
    magenta_s_min: int = 100
    magenta_s_max: int = 255
    magenta_v_min: int = 100
    magenta_v_max: int = 255

    # Detection
    min_area: int = 300

    def update(self, **kwargs):
        for key, value in kwargs.items():
            if hasattr(self, key):
                expected_type = type(getattr(self, key))
                setattr(self, key, expected_type(value))

    def save(self):
        with open(PARAMS_FILE, "w") as f:
            json.dump(asdict(self), f, indent=2)

    @classmethod
    def load(cls) -> "Parameters":
        if PARAMS_FILE.exists():
            with open(PARAMS_FILE) as f:
                data = json.load(f)
            params = cls()
            params.update(**data)
            return params
        return cls()
