"""
Web server - camera stream + decision visualization.

Stage 4: Added WorldState and decision info to the page.

New endpoints:
  GET  /api/world   -> returns current WorldState as JSON
  GET  /api/decision -> returns current decision (state, speed, steering)

The page now shows:
  - Camera stream (with bounding boxes as before)
  - Color masks (as before)
  - WorldState panel: walls, pillars, corner
  - Decision panel: current state, speed, steering
  - A diagram showing the full pipeline

This lets you see the ENTIRE chain:
  Camera pixels → blobs → fusion → WorldState → decision → (speed, steering)
"""

import asyncio
import json
from dataclasses import asdict
from pathlib import Path

from aiohttp import web

from params import Parameters
from world_state import WorldState

TEMPLATES_DIR = Path(__file__).parent / "templates"


class WebServer:
    """Web server showing the full perception-to-decision pipeline."""

    def __init__(self, camera, params, fusion, decision):
        self.camera = camera
        self.params = params
        self.fusion = fusion
        self.decision = decision
        self.app = web.Application()

        # Latest state (updated by background task)
        self._latest_world = WorldState()
        self._latest_speed = 0
        self._latest_steering = 90

        # Pages
        self.app.router.add_get("/", self.index)

        # Streams
        self.app.router.add_get("/stream/camera", self.stream_camera)
        self.app.router.add_get("/stream/camera/{color}", self.stream_camera_mask)

        # Parameters API (from Stage 3)
        self.app.router.add_get("/api/params", self.api_params_get)
        self.app.router.add_post("/api/params", self.api_params_set)

        # NEW: WorldState and Decision APIs
        self.app.router.add_get("/api/world", self.api_world_get)
        self.app.router.add_get("/api/decision", self.api_decision_get)

        # Start background fusion loop
        self.app.on_startup.append(self._start_fusion_loop)
        self.app.on_cleanup.append(self._stop_fusion_loop)
        self._fusion_task = None

    async def _start_fusion_loop(self, app):
        """Run sensor fusion in the background at ~10 Hz."""
        self._fusion_task = asyncio.ensure_future(self._fusion_loop())

    async def _stop_fusion_loop(self, app):
        if self._fusion_task:
            self._fusion_task.cancel()
            try:
                await self._fusion_task
            except asyncio.CancelledError:
                pass

    async def _fusion_loop(self):
        """Background loop: fuse sensors → decide → store results.

        This mimics what the real robot's control loop does:
          1. Fuse sensor data → WorldState
          2. Decide speed/steering from WorldState
          3. (Real robot would send to motor here)
          4. We just store the results for the web UI to display
        """
        while True:
            try:
                if self.camera.is_running:
                    # Step 1: Sensor fusion
                    world = self.fusion.update()
                    self._latest_world = world

                    # Step 2: Decision
                    speed, steering = self.decision.decide(world)
                    self._latest_speed = speed
                    self._latest_steering = steering

            except Exception as e:
                print(f"Fusion error: {e}")

            await asyncio.sleep(0.1)  # 10 Hz

    # ── Pages ─────────────────────────────────────────────

    async def index(self, request):
        html = (TEMPLATES_DIR / "camera.html").read_text()
        return web.Response(text=html, content_type="text/html")

    # ── Streams ───────────────────────────────────────────

    async def stream_camera(self, request):
        response = web.StreamResponse()
        response.content_type = "multipart/x-mixed-replace; boundary=frame"
        await response.prepare(request)
        try:
            while True:
                if self.camera.is_running:
                    jpeg = self.camera.get_jpeg_frame()
                    if jpeg:
                        await response.write(
                            b"--frame\r\n"
                            b"Content-Type: image/jpeg\r\n\r\n"
                            + jpeg + b"\r\n"
                        )
                await asyncio.sleep(0.05)
        except (ConnectionResetError, ConnectionAbortedError):
            pass
        return response

    async def stream_camera_mask(self, request):
        color = request.match_info["color"]
        if color not in ("red", "green", "magenta"):
            return web.Response(status=404, text="Unknown color")
        response = web.StreamResponse()
        response.content_type = "multipart/x-mixed-replace; boundary=frame"
        await response.prepare(request)
        try:
            while True:
                if self.camera.is_running:
                    jpeg = self.camera.get_jpeg_mask(color)
                    if jpeg:
                        await response.write(
                            b"--frame\r\n"
                            b"Content-Type: image/jpeg\r\n\r\n"
                            + jpeg + b"\r\n"
                        )
                await asyncio.sleep(0.05)
        except (ConnectionResetError, ConnectionAbortedError):
            pass
        return response

    # ── Parameters API ────────────────────────────────────

    async def api_params_get(self, request):
        return web.json_response(asdict(self.params))

    async def api_params_set(self, request):
        data = await request.json()
        save = data.pop("_save", False)
        self.params.update(**data)
        if save:
            self.params.save()
        return web.json_response(asdict(self.params))

    # ── WorldState API (NEW) ──────────────────────────────

    async def api_world_get(self, request):
        """GET /api/world - Return current WorldState.

        This lets the browser see exactly what the robot "knows"
        about the world right now.
        """
        world = self._latest_world
        return web.json_response({
            "walls": {
                "left": world.walls.left_distance,
                "right": world.walls.right_distance,
                "front": world.walls.front_distance,
                "corridor_width": world.walls.corridor_width,
            },
            "pillars": [
                {
                    "color": p.color,
                    "angle": round(p.angle, 1),
                    "distance": round(p.distance, 0),
                    "pass_side": p.pass_side,
                    "blocking": p.is_blocking(),
                }
                for p in world.pillars
            ],
            "corner_ahead": world.corner_ahead,
            "has_pillars": world.has_pillars,
            "is_corner": world.is_corner_approaching,
        })

    async def api_decision_get(self, request):
        """GET /api/decision - Return current decision output.

        Shows what the state machine decided to do.
        """
        return web.json_response({
            "state": self.decision.state.name,
            "speed": self._latest_speed,
            "steering": self._latest_steering,
        })


async def run_server(camera, params, fusion, decision, host="0.0.0.0", port=8080):
    """Start the web server."""
    server = WebServer(camera, params, fusion, decision)
    runner = web.AppRunner(server.app)
    await runner.setup()
    site = web.TCPSite(runner, host, port)
    await site.start()
    print(f"Web server running at http://{host}:{port}")
    return runner
