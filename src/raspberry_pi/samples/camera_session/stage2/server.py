"""
Web server - streams camera and color masks to browser.

Stage 3: Camera stream with bounding boxes + color mask streams.
"""

import asyncio
from pathlib import Path

from aiohttp import web

TEMPLATES_DIR = Path(__file__).parent / "templates"


class WebServer:
    """Web server with camera stream and color mask streams."""

    def __init__(self, camera):
        self.camera = camera
        self.app = web.Application()

        # Register routes
        self.app.router.add_get("/", self.index)
        self.app.router.add_get("/stream/camera", self.stream_camera)
        self.app.router.add_get("/stream/camera/{color}", self.stream_camera_mask)

    async def index(self, request):
        """Serve the HTML page."""
        html = (TEMPLATES_DIR / "camera.html").read_text()
        return web.Response(text=html, content_type="text/html")

    async def stream_camera(self, request):
        """MJPEG stream of camera with bounding boxes drawn on detected blobs."""
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
                            + jpeg
                            + b"\r\n"
                        )
                await asyncio.sleep(0.05)  # ~20 FPS
        except (ConnectionResetError, ConnectionAbortedError):
            pass

        return response

    async def stream_camera_mask(self, request):
        """MJPEG stream of a color mask (red, green, or magenta).

        Shows white pixels where the color is detected, black everywhere else.
        Useful for tuning HSV ranges.
        """
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
                            + jpeg
                            + b"\r\n"
                        )
                await asyncio.sleep(0.05)  # ~20 FPS
        except (ConnectionResetError, ConnectionAbortedError):
            pass

        return response


async def run_server(camera, host="0.0.0.0", port=8080):
    """Start the web server."""
    server = WebServer(camera)
    runner = web.AppRunner(server.app)
    await runner.setup()
    site = web.TCPSite(runner, host, port)
    await site.start()
    print(f"Web server running at http://{host}:{port}")
    return runner
