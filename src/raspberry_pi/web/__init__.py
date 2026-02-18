"""
Web Layer - Debug and remote control interface.

Provides:
- LIDAR visualization (WebSocket + p5.js)
- Camera view (MJPEG stream)
- Manual motor control
- Parameter tuning
- Log viewer

NOTE: Must be OFF during competition (WRO rules prohibit wireless).
"""

from .server import create_app, run_server

__all__ = ["create_app", "run_server"]
