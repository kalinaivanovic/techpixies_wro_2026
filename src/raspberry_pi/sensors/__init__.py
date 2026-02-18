"""
Sensor Layer - Hardware interfaces.

Provides access to all robot sensors:
- Lidar: RPLIDAR C1 for distance sensing
- Camera: IMX219-120 for color detection
- Motor: ESP32 communication for motor control and encoder reading
"""

from .lidar import Lidar
from .camera import Camera
from .motor import Motor

__all__ = ["Lidar", "Camera", "Motor"]
