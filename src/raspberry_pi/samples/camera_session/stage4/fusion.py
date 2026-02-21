"""
Sensor Fusion - Combines camera and LIDAR into WorldState.

Stage 4: This is where the magic happens.

The camera knows COLORS but not DISTANCES.
The LIDAR knows DISTANCES but not COLORS.
Sensor fusion combines them:

    Camera:  "Red blob at angle +15°"
    LIDAR:   "Object at angle +14°, distance 350mm, width 55mm"
                              ↓
    Fusion:  Angles match (15° ≈ 14°)? YES.
             Object size matches pillar (50mm)? YES.
                              ↓
    Result:  Pillar(color="red", angle=15°, distance=350mm)

The matching algorithm:
  1. Get camera blobs (red and green only — we ignore magenta here)
  2. Get LIDAR objects (clustered groups of scan points)
  3. For each camera blob, find the LIDAR object at the closest angle
  4. If the angle difference is small enough AND the object is pillar-sized:
     → It's a confirmed pillar!
  5. If no LIDAR match → might be a false positive, ignore it

Why do we need BOTH sensors?
  - Camera alone: sees "red" but could be a spectator's jacket
  - LIDAR alone: sees "small object" but doesn't know the color
  - Both together: "small object that is red" → definitely a pillar

In this educational demo, we SIMULATE LIDAR data so you can run this
with just a camera. Real LIDAR integration is in the production code.
"""

import time
from typing import Optional
from dataclasses import dataclass

from camera import Camera, ColorBlob
from world_state import WorldState, WallInfo, Pillar


@dataclass
class SimulatedLidarObject:
    """A fake LIDAR detection for educational purposes.

    In the real robot, this comes from LIDAR scan clustering.
    Here we create fake objects from camera blobs so you can
    see the fusion pipeline working with just a camera.
    """

    angle: float      # Degrees from center
    distance: float   # mm (simulated)
    width: float      # mm (simulated physical width)


class SensorFusion:
    """
    Combines camera blobs with LIDAR objects to produce WorldState.

    In this educational version, we simulate LIDAR data from camera blobs.
    The REAL production code (perception/sensor_fusion.py) uses actual
    LIDAR scan data from the RPLIDAR C1.

    The ALGORITHM is the same though:
      1. Get camera blobs
      2. Get LIDAR objects (simulated here, real in production)
      3. Match by angle
      4. Create Pillar objects for confirmed matches
      5. Package everything into WorldState

    Usage:
        camera = Camera(params)
        fusion = SensorFusion(camera)

        # In a loop:
        world = fusion.update()
        if world.has_pillars:
            print(f"Pillar: {world.closest_pillar}")
    """

    # How close in degrees must camera and LIDAR agree?
    ANGLE_MATCH_THRESHOLD = 15.0

    # Pillar size range in mm (WRO pillars are 50x50x100mm)
    PILLAR_SIZE_MIN = 20
    PILLAR_SIZE_MAX = 120

    def __init__(self, camera: Camera):
        self.camera = camera

    def update(self) -> WorldState:
        """Fuse sensor data into WorldState.

        This is called every control loop iteration (~20-50 Hz).

        Steps:
          1. Get what the camera sees (colored blobs)
          2. Get what the LIDAR sees (objects with distances)
          3. Match them by angle → confirmed pillars
          4. Build WorldState with all perception data
        """
        # Step 1: What does the camera see?
        blobs = self.camera.get_blobs()

        # Step 2: What does the LIDAR see?
        # (In production: self.lidar.get_objects())
        # (Here: we simulate from camera data for the demo)
        lidar_objects = self._simulate_lidar(blobs)

        # Step 3: Match camera colors with LIDAR objects
        pillars = self._match_pillars(lidar_objects, blobs)

        # Step 4: Build WorldState
        # (Wall info would come from LIDAR too - simulated here)
        walls = WallInfo(
            left_distance=500.0,   # Simulated: 500mm to left wall
            right_distance=500.0,  # Simulated: 500mm to right wall
            front_distance=2000.0, # Simulated: 2m to front wall
        )

        return WorldState(
            walls=walls,
            pillars=pillars,
            corner_ahead=None,
        )

    def _match_pillars(
        self,
        objects: list[SimulatedLidarObject],
        blobs: list[ColorBlob],
    ) -> list[Pillar]:
        """Match camera blobs with LIDAR objects by angle.

        This is the core fusion algorithm:

        For each camera blob (red or green):
          - Look through all LIDAR objects
          - Find the one with the closest angle
          - If angle difference < threshold AND object is pillar-sized:
            → Create a confirmed Pillar

        Example:
          Camera blob: color="red", angle=+15°
          LIDAR objects: [
            {angle=+14°, distance=350mm, width=55mm},  ← match!
            {angle=-30°, distance=800mm, width=400mm},  ← too far in angle
          ]
          Result: Pillar(color="red", angle=15°, distance=350mm)
        """
        pillars = []
        used_objects = set()

        # Only try to match red and green blobs (not magenta)
        pillar_blobs = [b for b in blobs if b.color in ("red", "green")]

        for blob in pillar_blobs:
            best_match = None
            best_angle_diff = float("inf")

            for i, obj in enumerate(objects):
                if i in used_objects:
                    continue

                # Is this object pillar-sized?
                if not (self.PILLAR_SIZE_MIN <= obj.width <= self.PILLAR_SIZE_MAX):
                    continue

                # How close is the angle?
                angle_diff = abs(blob.angle - obj.angle)

                if angle_diff < self.ANGLE_MATCH_THRESHOLD and angle_diff < best_angle_diff:
                    best_match = (i, obj)
                    best_angle_diff = angle_diff

            # If we found a matching LIDAR object → confirmed pillar!
            if best_match is not None:
                idx, obj = best_match
                used_objects.add(idx)
                pillars.append(
                    Pillar(
                        color=blob.color,
                        angle=blob.angle,
                        distance=obj.distance,
                    )
                )

        return pillars

    def _simulate_lidar(self, blobs: list[ColorBlob]) -> list[SimulatedLidarObject]:
        """Create fake LIDAR objects from camera blobs.

        In the real robot, LIDAR independently scans the environment.
        Here we cheat: if the camera sees a blob, we pretend the LIDAR
        also sees an object at the same angle with a made-up distance.

        We estimate distance from the blob's pixel area:
          - Big blob (5000 px) → close (~200mm)
          - Small blob (300 px) → far (~1500mm)

        This is a rough approximation! Real LIDAR gives exact distances.
        """
        objects = []
        for blob in blobs:
            if blob.color not in ("red", "green"):
                continue

            # Estimate distance from area (bigger blob = closer)
            # This is very approximate - real LIDAR is much more accurate
            estimated_distance = max(200, 2000 - blob.area * 0.3)

            objects.append(SimulatedLidarObject(
                angle=blob.angle,
                distance=estimated_distance,
                width=55.0,  # Assume pillar-sized (50mm real + margin)
            ))

        return objects
