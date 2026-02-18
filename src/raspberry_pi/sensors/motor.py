"""
Motor sensor/actuator - ESP32 communication.

Handles:
- Sending speed and steering commands to ESP32
- Reading encoder values
- Emergency stop
"""

import logging
from typing import Optional

import serial

from config import ESP32_PORT, ESP32_BAUDRATE, STEERING_CENTER

logger = logging.getLogger(__name__)


class Motor:
    """
    ESP32 motor controller communication.

    Protocol:
        Commands (Pi -> ESP32):
            C:<speed>,<steer>\\n  - speed: -100..100, steer: 0..180
            E\\n                  - emergency stop
            R\\n                  - reset encoder

        Status (ESP32 -> Pi):
            S:<encoder>,<speed>,<steer>\\n
            E:<error_code>\\n
    """

    def __init__(self, port: str = ESP32_PORT, baudrate: int = ESP32_BAUDRATE):
        self.port = port
        self.baudrate = baudrate

        self._serial: Optional[serial.Serial] = None
        self._speed = 0
        self._steering = STEERING_CENTER
        self._encoder = 0
        self._connected = False

    @property
    def is_connected(self) -> bool:
        return self._connected

    @property
    def speed(self) -> int:
        return self._speed

    @property
    def steering(self) -> int:
        return self._steering

    @property
    def encoder(self) -> int:
        return self._encoder

    def connect(self) -> bool:
        """Open serial connection to ESP32."""
        try:
            self._serial = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=0.1
            )
            self._connected = True
            logger.info(f"Connected to ESP32 on {self.port}")
            return True
        except Exception as e:
            logger.error(f"Failed to connect to ESP32: {e}")
            self._connected = False
            return False

    def disconnect(self):
        """Close serial connection."""
        if self._serial:
            self.emergency_stop()
            self._serial.close()
            self._serial = None
        self._connected = False
        logger.info("Disconnected from ESP32")

    def drive(self, speed: int, steering: int):
        """
        Set speed and steering, then send to ESP32.

        Args:
            speed: -100 to 100 (negative = reverse)
            steering: 0 to 180 (90 = center)
        """
        self._speed = max(-100, min(100, speed))
        self._steering = max(0, min(180, steering))
        self._send_command()

    def stop(self):
        """Stop motors (speed = 0, maintain steering)."""
        self._speed = 0
        self._send_command()
        logger.info("Motors stopped")

    def emergency_stop(self):
        """Emergency stop - sends E command."""
        if self._serial:
            self._serial.write(b"E\n")
            self._speed = 0
            logger.warning("EMERGENCY STOP")

    def reset_encoder(self):
        """Reset encoder counter to zero."""
        if self._serial:
            self._serial.write(b"R\n")
            self._encoder = 0
            logger.info("Encoder reset")

    def update(self) -> bool:
        """
        Read status from ESP32 (non-blocking).

        Call this regularly to keep encoder value updated.

        Returns:
            True if status was received
        """
        if not self._serial or not self._serial.in_waiting:
            return False

        try:
            line = self._serial.readline().decode().strip()

            if line.startswith("S:"):
                # Status: S:<encoder>,<speed>,<steer>
                parts = line[2:].split(",")
                if len(parts) >= 3:
                    self._encoder = int(parts[0])
                return True

            elif line.startswith("E:"):
                # Error from ESP32
                error_code = line[2:]
                logger.error(f"ESP32 error: {error_code}")
                return True

        except Exception as e:
            logger.error(f"Error reading ESP32 status: {e}")

        return False

    def _send_command(self):
        """Send current speed and steering to ESP32."""
        if not self._serial:
            logger.warning("Not connected to ESP32")
            return

        command = f"C:{self._speed},{self._steering}\n"
        self._serial.write(command.encode())
        logger.debug(f"Sent: {command.strip()}")

    def __enter__(self):
        """Context manager entry."""
        self.connect()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        """Context manager exit."""
        self.disconnect()
