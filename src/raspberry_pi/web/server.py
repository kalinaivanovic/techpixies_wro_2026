"""
Web server - aiohttp application for debug interface.
"""

import asyncio
import json
import logging
import shutil
import subprocess
import threading
import time
from pathlib import Path
from typing import Optional

from aiohttp import web

from config import WEB_HOST, WEB_PORT

logger = logging.getLogger(__name__)

# Path to static files and templates
WEB_DIR = Path(__file__).parent
STATIC_DIR = WEB_DIR / "static"
TEMPLATES_DIR = WEB_DIR / "templates"


class WebServer:
    """
    Debug web interface server.

    Provides:
    - Dashboard page
    - LIDAR visualization with WebSocket
    - Motor control with WebSocket
    - Camera stream (MJPEG)
    """

    def __init__(self, controller=None):
        """
        Args:
            controller: Optional Controller instance for live data
        """
        self.controller = controller
        self.app = web.Application()
        self._keepalive_thread: Optional[threading.Thread] = None
        self._keepalive_running = False
        self._setup_routes()
        self.app.on_startup.append(self._on_startup)
        self.app.on_cleanup.append(self._on_cleanup)

    def _setup_routes(self):
        """Configure routes."""
        # Pages
        self.app.router.add_get("/", self.index)
        self.app.router.add_get("/lidar", self.lidar_page)
        self.app.router.add_get("/controls", self.controls_page)
        self.app.router.add_get("/architecture", self.architecture_page)

        # API
        self.app.router.add_get("/api/status", self.api_status)

        # Streams
        self.app.router.add_get("/stream/camera", self.stream_camera)
        self.app.router.add_get("/stream/camera/{color}", self.stream_camera_mask)
        self.app.router.add_get("/stream/lidar", self.stream_lidar)
        self.app.router.add_get("/stream/lidar/raw", self.stream_lidar_raw)
        self.app.router.add_get("/stream/worldstate", self.stream_worldstate)
        self.app.router.add_get("/stream/worldstate/camera", self.stream_worldstate_camera)

        # LIDAR display params
        self.app.router.add_get("/api/lidar/params", self.api_lidar_params_get)
        self.app.router.add_post("/api/lidar/params", self.api_lidar_params_set)

        # WorldState API
        self.app.router.add_get("/api/worldstate", self.api_worldstate)

        # Motor control (REST)
        self.app.router.add_post("/api/drive", self.api_drive)
        self.app.router.add_post("/api/stop", self.api_stop)

        # Runtime parameters (HSV ranges, detection thresholds)
        self.app.router.add_get("/api/params", self.api_params_get)
        self.app.router.add_post("/api/params", self.api_params_set)

        # System info (Pi-specific, optional)
        self.app.router.add_get("/api/system", self.api_system)

        # WebSocket
        self.app.router.add_get("/ws/lidar", self.ws_lidar)
        self.app.router.add_get("/ws/control", self.ws_control)

        # Static files
        if STATIC_DIR.exists():
            self.app.router.add_static("/static", STATIC_DIR)

    async def index(self, request):
        """Dashboard page."""
        html = self._render_template("index.html")
        return web.Response(text=html, content_type="text/html")

    async def lidar_page(self, request):
        """LIDAR visualization page."""
        html = self._render_template("lidar.html")
        return web.Response(text=html, content_type="text/html")

    async def controls_page(self, request):
        """Motor control page."""
        html = self._render_template("controls.html")
        return web.Response(text=html, content_type="text/html")

    async def architecture_page(self, request):
        """Architecture visualization page."""
        html = self._render_template("architecture.html")
        return web.Response(text=html, content_type="text/html")

    async def api_status(self, request):
        """Get current robot status."""
        status = {
            "state": "unknown",
            "lap": 0,
            "encoder": 0,
            "speed": 0,
            "steering": 90,
        }

        if self.controller:
            try:
                status["state"] = self.controller.state_machine.state.name
                status["lap"] = self.controller.state_machine.lap_count
                status["encoder"] = self.controller.motor.encoder
                status["speed"] = self.controller.motor.speed
                status["steering"] = self.controller.motor.steering
            except AttributeError:
                pass  # Not all components available (e.g. camera-only mode)

        return web.json_response(status)

    async def api_lidar_params_get(self, request):
        """Get current LIDAR display parameters."""
        if self.controller and hasattr(self.controller, 'lidar') and self.controller.lidar:
            lidar = self.controller.lidar
            return web.json_response({
                "max_distance": lidar.display_max_distance,
                "angle": lidar.display_angle,
                "min_quality": lidar.display_min_quality,
            })
        return web.json_response({"error": "LIDAR not available"}, status=404)

    async def api_lidar_params_set(self, request):
        """Set LIDAR display parameters."""
        if not self.controller or not hasattr(self.controller, 'lidar') or not self.controller.lidar:
            return web.json_response({"error": "LIDAR not available"}, status=404)

        data = await request.json()
        lidar = self.controller.lidar

        if "max_distance" in data:
            lidar.display_max_distance = int(data["max_distance"])
        if "angle" in data:
            lidar.display_angle = int(data["angle"])
        if "min_quality" in data:
            lidar.display_min_quality = int(data["min_quality"])

        return web.json_response({
            "max_distance": lidar.display_max_distance,
            "angle": lidar.display_angle,
            "min_quality": lidar.display_min_quality,
        })

    async def stream_camera(self, request):
        """MJPEG stream of camera with bounding boxes."""
        response = web.StreamResponse()
        response.content_type = "multipart/x-mixed-replace; boundary=frame"
        await response.prepare(request)

        try:
            while True:
                if self.controller and self.controller.camera.is_running:
                    jpeg = self.controller.camera.get_jpeg_frame()
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
        except Exception as e:
            logger.error(f"Camera stream error: {e}")

        return response

    async def stream_camera_mask(self, request):
        """MJPEG stream of a color mask (red, green, magenta)."""
        color = request.match_info["color"]
        if color not in ("red", "green", "magenta"):
            return web.Response(status=404, text="Unknown color")

        response = web.StreamResponse()
        response.content_type = "multipart/x-mixed-replace; boundary=frame"
        await response.prepare(request)

        try:
            while True:
                if self.controller and self.controller.camera.is_running:
                    jpeg = self.controller.camera.get_jpeg_mask(color)
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
        except Exception as e:
            logger.error(f"Camera mask stream error: {e}")

        return response

    async def stream_lidar(self, request):
        """MJPEG stream of LIDAR bird's eye view with clusters."""
        response = web.StreamResponse()
        response.content_type = "multipart/x-mixed-replace; boundary=frame"
        await response.prepare(request)

        try:
            while True:
                if self.controller and hasattr(self.controller, 'lidar') and self.controller.lidar.is_running:
                    jpeg = self.controller.lidar.get_jpeg_frame()
                    if jpeg:
                        await response.write(
                            b"--frame\r\n"
                            b"Content-Type: image/jpeg\r\n\r\n"
                            + jpeg
                            + b"\r\n"
                        )
                await asyncio.sleep(0.1)  # ~10 FPS (matches LIDAR scan rate)
        except (ConnectionResetError, ConnectionAbortedError):
            pass
        except Exception as e:
            logger.error(f"LIDAR stream error: {e}")

        return response

    async def stream_lidar_raw(self, request):
        """MJPEG stream of raw LIDAR points (no clustering)."""
        response = web.StreamResponse()
        response.content_type = "multipart/x-mixed-replace; boundary=frame"
        await response.prepare(request)

        try:
            while True:
                if self.controller and hasattr(self.controller, 'lidar') and self.controller.lidar.is_running:
                    jpeg = self.controller.lidar.get_jpeg_raw()
                    if jpeg:
                        await response.write(
                            b"--frame\r\n"
                            b"Content-Type: image/jpeg\r\n\r\n"
                            + jpeg
                            + b"\r\n"
                        )
                await asyncio.sleep(0.1)  # ~10 FPS
        except (ConnectionResetError, ConnectionAbortedError):
            pass
        except Exception as e:
            logger.error(f"LIDAR raw stream error: {e}")

        return response

    async def stream_worldstate(self, request):
        """MJPEG stream of bird's eye WorldState view."""
        response = web.StreamResponse()
        response.content_type = "multipart/x-mixed-replace; boundary=frame"
        await response.prepare(request)

        try:
            while True:
                if self.controller and hasattr(self.controller, 'visualizer'):
                    state_name = ""
                    lap = 0
                    direction = ""
                    try:
                        state_name = self.controller.state_machine.state.name
                        lap = self.controller.state_machine.lap_count
                        direction = self.controller.state_machine.direction or ""
                    except AttributeError:
                        pass

                    jpeg = self.controller.visualizer.render_birdseye(
                        self.controller.world_state,
                        self.controller.latest_scan,
                        state_name=state_name,
                        lap=lap,
                        direction=direction,
                    )
                    if jpeg:
                        await response.write(
                            b"--frame\r\n"
                            b"Content-Type: image/jpeg\r\n\r\n"
                            + jpeg
                            + b"\r\n"
                        )
                await asyncio.sleep(0.1)  # ~10 FPS
        except (ConnectionResetError, ConnectionAbortedError):
            pass
        except Exception as e:
            logger.error(f"WorldState stream error: {e}")

        return response

    async def stream_worldstate_camera(self, request):
        """MJPEG stream of camera with fusion annotations."""
        response = web.StreamResponse()
        response.content_type = "multipart/x-mixed-replace; boundary=frame"
        await response.prepare(request)

        try:
            while True:
                if self.controller and hasattr(self.controller, 'visualizer'):
                    state_name = ""
                    lap = 0
                    try:
                        state_name = self.controller.state_machine.state.name
                        lap = self.controller.state_machine.lap_count
                    except AttributeError:
                        pass

                    frame = self.controller.camera.get_frame() if self.controller.camera.is_running else None
                    jpeg = self.controller.visualizer.render_camera(
                        self.controller.world_state,
                        frame,
                        self.controller.latest_blobs,
                        state_name=state_name,
                        lap=lap,
                    )
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
        except Exception as e:
            logger.error(f"WorldState camera stream error: {e}")

        return response

    async def api_worldstate(self, request):
        """Get current WorldState as JSON."""
        data = {
            "walls": {"left": None, "right": None, "front": None},
            "corridor_width": None,
            "corner_ahead": None,
            "pillars": [],
            "parking_marker": None,
            "encoder_pos": 0,
            "state": "unknown",
            "lap": 0,
            "direction": None,
        }

        if self.controller:
            world = self.controller.world_state
            if world is not None:
                data["walls"] = {
                    "left": round(world.walls.left_distance) if world.walls.left_distance is not None else None,
                    "right": round(world.walls.right_distance) if world.walls.right_distance is not None else None,
                    "front": round(world.walls.front_distance) if world.walls.front_distance is not None else None,
                }
                data["corridor_width"] = round(world.corridor_width) if world.corridor_width is not None else None
                data["corner_ahead"] = world.corner_ahead
                data["pillars"] = [
                    {"color": p.color, "angle": round(p.angle, 1), "distance": round(p.distance)}
                    for p in world.pillars
                ]
                data["parking_marker"] = round(world.parking_marker) if world.parking_marker is not None else None
                data["encoder_pos"] = world.encoder_pos

            try:
                data["state"] = self.controller.state_machine.state.name
                data["lap"] = self.controller.state_machine.lap_count
                data["direction"] = self.controller.state_machine.direction
            except AttributeError:
                pass

        return web.json_response(data)

    async def api_system(self, request):
        """Get Pi system info (throttle state, temperature). No-op on non-Pi."""
        data = {}

        if shutil.which("vcgencmd"):
            try:
                result = subprocess.run(
                    ["vcgencmd", "get_throttled"],
                    capture_output=True, text=True, timeout=2,
                )
                if result.returncode == 0:
                    # Output: "throttled=0x50005"
                    val = result.stdout.strip().split("=")[-1]
                    flags = int(val, 16)
                    data["throttled_raw"] = val
                    data["undervoltage_now"] = bool(flags & 0x1)
                    data["throttled_now"] = bool(flags & 0x4)
                    data["undervoltage_occurred"] = bool(flags & 0x10000)
                    data["throttled_occurred"] = bool(flags & 0x40000)
            except Exception:
                pass

            try:
                result = subprocess.run(
                    ["vcgencmd", "measure_temp"],
                    capture_output=True, text=True, timeout=2,
                )
                if result.returncode == 0:
                    # Output: "temp=42.0'C"
                    temp_str = result.stdout.strip().split("=")[-1].rstrip("'C")
                    data["cpu_temp"] = float(temp_str)
            except Exception:
                pass

        return web.json_response(data)

    async def api_params_get(self, request):
        """Get current tunable parameters."""
        if self.controller and hasattr(self.controller, 'params') and self.controller.params:
            return web.json_response(self.controller.params.to_dict())
        return web.json_response({"error": "Parameters not available"}, status=404)

    async def api_params_set(self, request):
        """Update tunable parameters. Include _save=true to persist to disk."""
        if not self.controller or not hasattr(self.controller, 'params') or not self.controller.params:
            return web.json_response({"error": "Parameters not available"}, status=404)

        data = await request.json()
        save = data.pop("_save", False)
        self.controller.params.update(**data)

        if save:
            self.controller.params.save()

        return web.json_response(self.controller.params.to_dict())

    async def ws_lidar(self, request):
        """WebSocket for LIDAR data streaming."""
        ws = web.WebSocketResponse()
        await ws.prepare(request)

        logger.info("LIDAR WebSocket connected")

        try:
            while not ws.closed:
                if self.controller and self.controller.lidar.is_running:
                    scan = self.controller.lidar.get_scan()
                    # Convert to list of {angle, distance} for JSON
                    points = [
                        {"angle": a, "distance": d}
                        for a, d in scan.items()
                    ]
                    await ws.send_json({"points": points})
                await asyncio.sleep(0.1)  # 10 Hz update
        except Exception as e:
            logger.error(f"LIDAR WebSocket error: {e}")
        finally:
            logger.info("LIDAR WebSocket disconnected")

        return ws

    async def ws_control(self, request):
        """WebSocket for motor control."""
        ws = web.WebSocketResponse()
        await ws.prepare(request)

        logger.info("Control WebSocket connected")

        try:
            async for msg in ws:
                if msg.type == web.WSMsgType.TEXT:
                    try:
                        data = json.loads(msg.data)
                        cmd = data.get("cmd")

                        if cmd == "drive" and self.controller:
                            speed = int(data.get("speed", 0))
                            steering = int(data.get("steering", 90))
                            self.controller.motor.drive(speed, steering)
                            await ws.send_json({"ok": True})

                        elif cmd == "stop" and self.controller:
                            self.controller.motor.stop()
                            await ws.send_json({"ok": True})

                        elif cmd == "status":
                            await ws.send_json({
                                "speed": self.controller.motor.speed if self.controller else 0,
                                "steering": self.controller.motor.steering if self.controller else 90,
                                "encoder": self.controller.motor.encoder if self.controller else 0,
                            })

                    except Exception as e:
                        await ws.send_json({"error": str(e)})

        except Exception as e:
            logger.error(f"Control WebSocket error: {e}")
        finally:
            logger.info("Control WebSocket disconnected")

        return ws

    # --- Motor REST endpoints ---

    def _get_motor(self):
        """Get motor instance if available."""
        if self.controller and hasattr(self.controller, 'motor') and self.controller.motor:
            return self.controller.motor
        return None

    async def api_drive(self, request):
        """POST /api/drive - Set target speed/steering (keepalive thread sends to ESP32)."""
        motor = self._get_motor()
        if not motor:
            return web.json_response({"error": "Motor not available"}, status=404)

        data = await request.json()
        speed = int(data.get("speed", 0))
        steering = int(data.get("steering", 90))
        # Only set target values — keepalive thread does the actual serial write
        motor._speed = max(-100, min(100, speed))
        motor._steering = max(0, min(180, steering))
        return web.json_response({
            "ok": True,
            "speed": motor.speed,
            "steering": motor.steering,
        })

    async def api_stop(self, request):
        """POST /api/stop - Emergency stop."""
        motor = self._get_motor()
        if not motor:
            return web.json_response({"error": "Motor not available"}, status=404)

        motor._speed = 0
        motor._steering = 90
        return web.json_response({"ok": True, "speed": 0})

    # --- Motor keepalive ---

    async def _on_startup(self, app):
        """Start background tasks."""
        self._keepalive_running = True
        self._keepalive_thread = threading.Thread(
            target=self._motor_keepalive_loop, daemon=True
        )
        self._keepalive_thread.start()

    async def _on_cleanup(self, app):
        """Stop background tasks."""
        self._keepalive_running = False
        if self._keepalive_thread:
            self._keepalive_thread.join(timeout=1.0)

    def _motor_keepalive_loop(self):
        """Dedicated thread: re-send current command every 20ms to feed ESP32 watchdog."""
        while self._keepalive_running:
            motor = self._get_motor()
            if motor and motor.is_connected:
                # Send current values without overwriting them (avoid race with REST handler)
                motor._send_command()
                # Drain serial buffer
                while motor._serial and motor._serial.in_waiting:
                    motor.update()
            time.sleep(0.02)

    def _render_template(self, name: str) -> str:
        """Render a template file."""
        template_path = TEMPLATES_DIR / name
        if template_path.exists():
            return template_path.read_text()

        # Fallback if template doesn't exist
        return f"""
        <!DOCTYPE html>
        <html>
        <head><title>WRO Robot - {name}</title></head>
        <body>
            <h1>WRO 2025 Robot Debug Interface</h1>
            <p>Template '{name}' not found. Create it at:</p>
            <pre>{template_path}</pre>
            <nav>
                <a href="/">Dashboard</a> |
                <a href="/lidar">LIDAR</a> |
                <a href="/controls">Controls</a>
            </nav>
        </body>
        </html>
        """


def create_app(controller=None) -> web.Application:
    """Create the web application."""
    server = WebServer(controller)
    return server.app


async def run_server(controller=None, host=WEB_HOST, port=WEB_PORT):
    """Run the web server."""
    app = create_app(controller)
    runner = web.AppRunner(app)
    await runner.setup()
    site = web.TCPSite(runner, host, port)
    await site.start()
    logger.info(f"Web server running at http://{host}:{port}")
    return runner


if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)
    web.run_app(create_app(), host=WEB_HOST, port=WEB_PORT)
