"""
Web server - streams camera to browser via MJPEG.

Stage 1: Just one page with camera stream.
"""

import asyncio
from pathlib import Path

from aiohttp import web

# Where templates live (same directory as this file)
TEMPLATES_DIR = Path(__file__).parent / "templates"


class WebServer:
    """Minimal web server: serves a page with camera stream."""

    def __init__(self, camera):
        self.camera = camera
        self.app = web.Application()

        # Register routes
        self.app.router.add_get("/", self.index)
        self.app.router.add_get("/stream/camera", self.stream_camera)

    async def index(self, request):
        """Serve the HTML page."""
        html = (TEMPLATES_DIR / "camera.html").read_text()
        return web.Response(text=html, content_type="text/html")

    async def stream_camera(self, request):
        """MJPEG stream: sends JPEG frames continuously.

        The browser displays this with a simple <img src="/stream/camera"> tag.
        Content-Type 'multipart/x-mixed-replace' tells the browser:
        "I will keep sending new images, replace the old one each time."
        """
        response = web.StreamResponse()
        response.content_type = "multipart/x-mixed-replace; boundary=frame"
        await response.prepare(request)

        try:
            while True:
                if self.camera.is_running:
                    jpeg = self.camera.get_jpeg_frame()
                    if jpeg:
                        # Each frame is sent as a MIME part with boundary
                        await response.write(
                            b"--frame\r\n"
                            b"Content-Type: image/jpeg\r\n\r\n"
                            + jpeg
                            + b"\r\n"
                        )
                await asyncio.sleep(0.05)  # ~20 frames per second
        except (ConnectionResetError, ConnectionAbortedError):
            pass  # Browser disconnected

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
