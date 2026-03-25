"""
Motor sensor/actuator - ESP32 communication.

Handles:
- Sending speed and steering commands to ESP32
- Reading encoder values
- Speed/RPM/distance calculation from encoder
- Emergency stop
"""

from __future__ import annotations

import math
import logging
import time
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

    def __init__(self, port: str = ESP32_PORT, baudrate: int = ESP32_BAUDRATE, params=None):
        self.port = port
        self.baudrate = baudrate
        self.params = params

        self._serial: serial.Serial | None = None
        self._speed = 0
        self._steering = STEERING_CENTER
        self._encoder = 0
        self._connected = False

        # Speed calculation from encoder deltas
        self._prev_encoder = 0
        self._prev_time = 0.0
        self._ticks_per_sec = 0.0
        self._rpm = 0.0
        self._speed_cm_s = 0.0
        self._distance_cm = 0.0
        self._total_ticks = 0  # Absolute ticks traveled (ignores direction)

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

    @property
    def ticks_per_sec(self) -> float:
        return self._ticks_per_sec

    @property
    def rpm(self) -> float:
        return self._rpm

    @property
    def speed_cm_s(self) -> float:
        return self._speed_cm_s

    @property
    def distance_cm(self) -> float:
        return self._distance_cm

    def connect(self) -> bool:
        """Open serial connection to ESP32."""
        try:
            self._serial = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=0.005
            )
            self._connected = True
            self.reset_encoder()
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
            line = self._serial.readline().decode(errors="ignore").strip()

            if line.startswith("S:"):
                # Status: S:<encoder>,<speed>,<steer>
                parts = line[2:].split(",")
                if len(parts) >= 3:
                    new_encoder = int(parts[0])
                    self._update_speed(new_encoder)
                    self._encoder = new_encoder
                return True

            elif line.startswith("E:"):
                # Error from ESP32
                error_code = line[2:]
                logger.error(f"ESP32 error: {error_code}")
                return True

        except Exception as e:
            logger.error(f"Error reading ESP32 status: {e}")
            # Flush the input buffer to avoid getting stuck in a read loop
            try:
                if self._serial:
                    self._serial.reset_input_buffer()
            except Exception:
                pass

        return False

    def _update_speed(self, new_encoder: int):
        """Calculate speed metrics from encoder delta."""
        now = time.monotonic()

        if self._prev_time == 0:
            # First reading — just initialize, can't calculate speed yet
            self._prev_encoder = new_encoder
            self._prev_time = now
            return

        dt = now - self._prev_time
        if dt > 0.005:  # At least 5ms between updates to avoid noise
            delta = new_encoder - self._prev_encoder
            self._total_ticks += abs(delta)

            raw_tps = delta / dt
            # Smooth with exponential moving average (0.3 new, 0.7 old)
            self._ticks_per_sec = 0.7 * self._ticks_per_sec + 0.3 * raw_tps

            self._prev_encoder = new_encoder
            self._prev_time = now

    def update_derived(self, params):
        """Recalculate RPM, speed, distance from params. Call from keepalive loop."""
        cpr = params.encoder_cpr
        wheel_d = params.wheel_diameter_mm

        if cpr > 0:
            self._rpm = (self._ticks_per_sec / cpr) * 60.0
            circumference_cm = math.pi * wheel_d / 10.0  # mm to cm
            self._speed_cm_s = (self._ticks_per_sec / cpr) * circumference_cm
            self._distance_cm = (self._total_ticks / cpr) * circumference_cm
        else:
            # Uncalibrated: show raw ticks
            self._rpm = 0
            self._speed_cm_s = 0
            self._distance_cm = 0

    def _send_command(self):
        """Send current speed and steering to ESP32."""
        if not self._serial:
            logger.warning("Not connected to ESP32")
            return

        # Servo is wired inverted: 0=right, 180=left. Flip to match
        # convention where low values=left, high values=right.
        hw_steering = 180 - self._steering
        command = f"C:{self._speed},{hw_steering}\n"
        self._serial.write(command.encode())
        logger.debug(f"Sent: {command.strip()}")

    def __enter__(self):
        """Context manager entry."""
        self.connect()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        """Context manager exit."""
        self.disconnect()
