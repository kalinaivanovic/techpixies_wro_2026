"""
Web server - camera stream + live parameter tuning.

Stage 3: Added REST API for reading/writing Parameters.

New endpoints:
  GET  /api/params     -> returns all params as JSON
  POST /api/params     -> updates params from JSON body
  POST /api/params with {"_save": true} -> also saves to disk

The flow:
  1. Browser loads page, fetches GET /api/params, populates sliders
  2. User moves a slider (e.g., Red H Min)
  3. JavaScript sends POST /api/params {"red_h_min": 5}
  4. Server calls params.update(red_h_min=5)
  5. Next camera frame reads params.red_h_min -> gets 5
  6. User sees the effect immediately in the MJPEG stream

No restart needed! The Camera reads params every frame.
"""

import asyncio
import json
from dataclasses import asdict
from pathlib import Path

from aiohttp import web

from params import Parameters

TEMPLATES_DIR = Path(__file__).parent / "templates"


class WebServer:
    """Web server with camera stream and parameter tuning."""

    def __init__(self, camera, params: Parameters):
        self.camera = camera
        self.params = params
        self.app = web.Application()

        # Pages
        self.app.router.add_get("/", self.index)

        # Streams
        self.app.router.add_get("/stream/camera", self.stream_camera)
        self.app.router.add_get("/stream/camera/{color}", self.stream_camera_mask)

        # Parameters API (NEW in Stage 3)
        self.app.router.add_get("/api/params", self.api_params_get)
        self.app.router.add_post("/api/params", self.api_params_set)

    async def index(self, request):
        html = (TEMPLATES_DIR / "camera.html").read_text()
        return web.Response(text=html, content_type="text/html")

    async def stream_camera(self, request):
        """MJPEG stream with bounding boxes."""
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
        """MJPEG stream of a single color mask."""
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

    # ── Parameters API ───────────────────────────────────────────

    async def api_params_get(self, request):
        """GET /api/params - Return all parameters as JSON."""
        return web.json_response(asdict(self.params))

    async def api_params_set(self, request):
        """POST /api/params - Update parameters from JSON body.

        Send {"red_h_min": 5, "green_h_max": 90} to change those values.
        Include "_save": true to also persist to params.json.
        """
        data = await request.json()

        # Pop special _save flag before updating params
        save = data.pop("_save", False)

        # Update the shared params object
        self.params.update(**data)

        # Optionally persist to disk
        if save:
            self.params.save()

        return web.json_response(asdict(self.params))


async def run_server(camera, params, host="0.0.0.0", port=8080):
    """Start the web server."""
    server = WebServer(camera, params)
    runner = web.AppRunner(server.app)
    await runner.setup()
    site = web.TCPSite(runner, host, port)
    await site.start()
    print(f"Web server running at http://{host}:{port}")
    return runner
