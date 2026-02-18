#!/usr/bin/env python3
"""
Stage 3: Camera with color detection in browser.

Run:
    python main.py

Then open http://localhost:8080 in your browser.
You will see the camera feed with bounding boxes around detected colors,
plus red and green mask views.
"""

import asyncio

from camera import Camera
from server import run_server


def main():
    camera = Camera()
    camera.start()

    async def run():
        runner = await run_server(camera)
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
