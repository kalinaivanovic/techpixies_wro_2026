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
                    self.lidar = Lidar(params=self.params)
                if use_motor:
                    from sensors.motor import Motor
                    self.motor = Motor(params=self.params)
                    self.motor.connect()

                self.visualizer = WorldStateVisualizer()
                self._fusion = None  # Created after sensors start

                # State machine: real if all sensors available, stub otherwise
                self._can_auto = use_lidar and use_motor
                if self._can_auto:
                    from decision import StateMachine
                    from perception import TrackMap
                    self.state_machine = StateMachine(params=self.params)
                    self.track_map = TrackMap()
                else:
                    class _StubState:
                        name = "IDLE"
                    class _StubSM:
                        state = _StubState()
                        lap_count = 0
                        direction = None
                    self.state_machine = _StubSM()
                    self.track_map = None

                # Auto mode state
                self._auto_running = False
                self._auto_task = None
                self._latest_world = None

            def init_fusion(self):
                """Create SensorFusion after sensors are started."""
                if self.lidar and self.camera:
                    encoder_fn = (lambda: self.motor.encoder) if self.motor else (lambda: 0)
                    self._fusion = SensorFusion(
                        self.lidar, self.camera, encoder_fn,
                    )

            @property
            def auto_running(self):
                return self._auto_running

            @property
            def world_state(self):
                if self._auto_running and self._latest_world:
                    return self._latest_world
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

            async def start_auto(self):
                """Start autonomous control loop."""
                if self._auto_running:
                    return
                if not self._can_auto:
                    raise RuntimeError("Need --motor and --lidar for auto mode")
                if not self._fusion:
                    raise RuntimeError("Sensor fusion not initialized")

                from decision import StateMachine
                from perception import TrackMap
                from config import CONTROL_LOOP_HZ

                # Fresh state machine and track map each run
                self.state_machine = StateMachine(params=self.params)
                self.track_map = TrackMap()
                self.state_machine.start()
                self.motor.reset_encoder()
                self._auto_running = True
                self._auto_task = asyncio.ensure_future(
                    self._auto_loop(CONTROL_LOOP_HZ)
                )
                logger.info("Auto mode started")

            async def stop_auto(self):
                """Stop autonomous control loop."""
                self._auto_running = False
                if self._auto_task:
                    self._auto_task.cancel()
                    try:
                        await self._auto_task
                    except asyncio.CancelledError:
                        pass
                    self._auto_task = None
                # Stop motor and center steering
                if self.motor:
                    self.motor.drive(0, 90)
                self._latest_world = None
                logger.info("Auto mode stopped")

            async def _auto_loop(self, hz):
                """Autonomous control loop (runs as asyncio task)."""
                from decision import RobotState

                period = 1.0 / hz
                loop_count = 0

                try:
                    while self._auto_running:
                        t0 = asyncio.get_event_loop().time()

                        # Perception
                        world = self._fusion.update()
                        self._latest_world = world

                        # Track mapping
                        if self.track_map:
                            self.track_map.update(world)

                        # Decision
                        speed, steering = self.state_machine.decide(
                            world, self.track_map
                        )

                        # Execute (set values; keepalive thread sends to ESP32)
                        if self.motor and self.motor.is_connected:
                            self.motor._speed = max(-100, min(100, speed))
                            self.motor._steering = max(0, min(180, steering))

                        # Race complete?
                        if self.state_machine.state == RobotState.DONE:
                            logger.info("Auto: race complete")
                            break

                        loop_count += 1
                        elapsed = asyncio.get_event_loop().time() - t0
                        await asyncio.sleep(max(0, period - elapsed))

                except asyncio.CancelledError:
                    pass
                except Exception as e:
                    logger.error(f"Auto loop error: {e}", exc_info=True)
                finally:
                    if self.motor:
                        self.motor.drive(0, 90)
                    self._auto_running = False

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
                if sensors.auto_running:
                    await sensors.stop_auto()
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
