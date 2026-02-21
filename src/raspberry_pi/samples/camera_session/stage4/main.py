#!/usr/bin/env python3
"""
Stage 4: From Pixels to Decisions.

This stage shows the FULL pipeline:

  Camera pixels → color blobs → sensor fusion → WorldState → decision → (speed, steering)

What's new from Stage 3:
  - SensorFusion: matches camera blobs with (simulated) LIDAR data
  - WorldState: the robot's combined understanding of the world
  - Decision: state machine that decides speed and steering
  - Web page: shows WorldState and decision output in real time

The web page displays:
  - Camera stream with bounding boxes (same as before)
  - Color masks (same as before)
  - WorldState panel: what the robot "knows"
  - Decision panel: what the robot "decides"
  - Pipeline diagram: data flow from camera to motor

Run:
    python main.py

Then open http://localhost:8080 in your browser.
Hold a red or green object in front of the camera and watch:
  1. Camera detects the color blob
  2. Fusion confirms it as a "pillar" (simulated LIDAR match)
  3. WorldState shows: "RED pillar at 500mm"
  4. Decision shows: "AVOID_PILLAR → steer LEFT, speed 30"
"""

import asyncio

from camera import Camera
from params import Parameters
from fusion import SensorFusion
from decision import Decision
from server import run_server


def main():
    # Layer 0: Parameters (shared config)
    params = Parameters.load()
    print(f"Parameters loaded: min_area={params.min_area}")

    # Layer 1: Sensor (camera)
    camera = Camera(params)
    camera.start()

    # Layer 2: Perception (sensor fusion)
    # In the real robot, this also takes a LIDAR object.
    # Here we simulate LIDAR from camera data.
    fusion = SensorFusion(camera)

    # Layer 3: Decision (state machine)
    decision = Decision()

    print()
    print("Pipeline ready:")
    print("  Camera → Fusion → WorldState → Decision → (speed, steering)")
    print()

    async def run():
        runner = await run_server(camera, params, fusion, decision)
        print("Press Ctrl+C to stop")
        try:
            while True:
                await asyncio.sleep(1)
        except asyncio.CancelledError:
            pass
        finally:
            camera.stop()
            await runner.cleanup()

    try:
        asyncio.run(run())
    except KeyboardInterrupt:
        camera.stop()


if __name__ == "__main__":
    main()
