"""
Perception Layer - World understanding.

Combines sensor data into a unified world model:
- SensorFusion: Fuses LIDAR + Camera → WorldState
- WorldState: Current instant perception
- TrackMap: Accumulated knowledge from lap 1
"""

from .world_state import WallInfo, Pillar, WorldState
from .sensor_fusion import SensorFusion
from .track_map import Corner, Section, PillarRecord, TrackMap

__all__ = [
    "WallInfo",
    "Pillar",
    "WorldState",
    "SensorFusion",
    "Corner",
    "Section",
    "PillarRecord",
    "TrackMap",
]
