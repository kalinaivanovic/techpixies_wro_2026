#!/usr/bin/env python3
"""
WRO 2025 Future Engineers - Main Entry Point

Usage:
    python main.py           # Run robot controller
    python main.py --web     # Run with web interface (debug mode)
"""

import argparse
import asyncio
import logging
import sys


def main():
    """Main entry point."""
    parser = argparse.ArgumentParser(description="WRO 2025 Robot Controller")
    parser.add_argument(
        "--web",
        action="store_true",
        help="Enable web interface for debugging",
    )
    parser.add_argument(
        "--lidar",
        action="store_true",
        help="Enable LIDAR in web mode",
    )
    parser.add_argument(
        "--motor",
        action="store_true",
        help="Enable motor controller (ESP32 serial)",
    )
    parser.add_argument(
        "--log-level",
        default="INFO",
        choices=["DEBUG", "INFO", "WARNING", "ERROR"],
        help="Logging level",
    )
    args = parser.parse_args()

    # Setup logging
    logging.basicConfig(
        level=getattr(logging, args.log_level),
        format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
    )

    logging.getLogger("aiohttp.access").setLevel(logging.WARNING)

    logger = logging.getLogger(__name__)
    logger.info("WRO 2025 Robot starting...")

    if args.web:
        # Run web interface with available sensors (no ESP32 needed)
        from params import Parameters
        from sensors.camera import Camera
        from perception.sensor_fusion import SensorFusion
        from perception.visualizer import WorldStateVisualizer
        from web.server import run_server

        class DebugSensors:
            def __init__(self, use_lidar=False, use_motor=False):
                self.params = Parameters.load()
                self.camera = Camera(params=self.params)
                self.lidar = None
                self.motor = None
                if use_lidar:
                    from sensors.lidar import Lidar
                    self.lidar = Lidar()
                if use_motor:
                    from sensors.motor import Motor
                    self.motor = Motor()
                    self.motor.connect()

                self.visualizer = WorldStateVisualizer()
                self._fusion = None  # Created after sensors start

                # Stub state_machine for API endpoints
                class _StubState:
                    name = "IDLE"
                class _StubSM:
                    state = _StubState()
                    lap_count = 0
                    direction = None
                self.state_machine = _StubSM()

            def init_fusion(self):
                """Create SensorFusion after sensors are started."""
                if self.lidar and self.camera:
                    self._fusion = SensorFusion(
                        self.lidar, self.camera, lambda: 0,
                    )

            @property
            def world_state(self):
                if self._fusion:
                    return self._fusion.update()
                return None

            @property
            def latest_scan(self):
                if self.lidar and self.lidar.is_running:
                    return self.lidar.get_scan()
                return {}

            @property
            def latest_blobs(self):
                if self.camera.is_running:
                    return self.camera.get_blobs()
                return []

        sensors = DebugSensors(use_lidar=args.lidar, use_motor=args.motor)
        sensors.camera.start()
        if sensors.lidar:
            logger.info("Starting LIDAR...")
            sensors.lidar.start()
        sensors.init_fusion()

        parts = ["camera"]
        if args.lidar:
            parts.append("LIDAR")
        if args.motor:
            parts.append("motor")
        logger.info(f"Web interface enabled ({' + '.join(parts)})")

        async def run_web():
            runner = await run_server(controller=sensors)
            logger.info("Press Ctrl+C to stop")
            try:
                while True:
                    await asyncio.sleep(1)
            except asyncio.CancelledError:
                pass
            finally:
                if sensors.motor:
                    sensors.motor.disconnect()
                sensors.camera.stop()
                if sensors.lidar:
                    sensors.lidar.stop()
                await runner.cleanup()

        asyncio.run(run_web())
    else:
        # Run full controller
        from control import Controller

        controller = Controller()
        asyncio.run(controller.run())


if __name__ == "__main__":
    main()
