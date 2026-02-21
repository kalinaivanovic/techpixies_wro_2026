#!/usr/bin/env python3
"""
Stage 3: Live parameter tuning in the browser.

What's new:
  - Parameters dataclass holds all HSV color ranges
  - Camera reads from params every frame (no hardcoded constants)
  - Web page has sliders to adjust HSV values
  - Changes take effect IMMEDIATELY (no restart needed!)

Run:
    python main.py

Then open http://localhost:8080 in your browser.
Move the sliders and watch the camera stream change in real time.
"""

import asyncio

from camera import Camera
from params import Parameters
from server import run_server


def main():
    # Load parameters (from params.json if it exists, else defaults)
    params = Parameters.load()
    print(f"Parameters loaded: min_area={params.min_area}")

    # Camera gets a REFERENCE to the same params object
    # When the web server changes params, the camera sees it next frame!
    camera = Camera(params)
    camera.start()

    async def run():
        # Server also gets the same params object
        runner = await run_server(camera, params)
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
