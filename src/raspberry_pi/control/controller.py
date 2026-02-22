"""
Main controller - Coordinates all layers.

This is the main control loop that:
1. Reads sensors
2. Fuses into WorldState
3. Updates TrackMap
4. Gets decision from StateMachine
5. Sends commands to motors
"""

import asyncio
import logging
import signal
import sys
from typing import Optional

from config import CONTROL_LOOP_HZ
from params import Parameters
from sensors import Lidar, Camera, Motor
from sensors.camera import ColorBlob
from perception import SensorFusion, TrackMap, WorldState
from perception.visualizer import WorldStateVisualizer
from decision import StateMachine, RobotState

logger = logging.getLogger(__name__)


class Controller:
    """
    Main robot controller.

    Coordinates:
    - Sensor layer (Lidar, Camera, Motor)
    - Perception layer (SensorFusion, TrackMap)
    - Decision layer (StateMachine)

    Usage:
        controller = Controller()
        asyncio.run(controller.run())
    """

    def __init__(self):
        # Runtime parameters (shared, tunable via web)
        self.params = Parameters.load()

        # Sensors
        self.lidar = Lidar(params=self.params)
        self.camera = Camera(params=self.params)
        self.motor = Motor(params=self.params)

        # Perception
        self.fusion: Optional[SensorFusion] = None
        self.track_map = TrackMap()

        # Decision
        self.state_machine = StateMachine(params=self.params)

        # Visualizer (for web debug)
        self.visualizer = WorldStateVisualizer()

        # Latest perception snapshot (for web access)
        self._latest_world: Optional[WorldState] = None
        self._latest_scan: dict[int, float] = {}
        self._latest_blobs: list[ColorBlob] = []

        # Control state
        self._running = False
        self._loop_count = 0

    @property
    def world_state(self) -> Optional[WorldState]:
        """Latest WorldState for web access."""
        return self._latest_world

    @property
    def latest_scan(self) -> dict[int, float]:
        """Latest raw LIDAR scan for web access."""
        return self._latest_scan

    @property
    def latest_blobs(self) -> list[ColorBlob]:
        """Latest camera blobs for web access."""
        return self._latest_blobs

    async def run(self):
        """Run the main control loop."""
        logger.info("Controller starting...")

        # Setup signal handlers for graceful shutdown
        loop = asyncio.get_event_loop()
        for sig in (signal.SIGINT, signal.SIGTERM):
            loop.add_signal_handler(sig, self._shutdown)

        try:
            # Initialize hardware
            if not self._init_hardware():
                logger.error("Failed to initialize hardware")
                return

            # Create fusion after sensors are ready
            self.fusion = SensorFusion(
                self.lidar,
                self.camera,
                lambda: self.motor.encoder,
            )

            # Start race
            self.state_machine.start()
            self._running = True

            logger.info("Entering main control loop")
            await self._control_loop()

        except Exception as e:
            logger.error(f"Controller error: {e}")
            raise
        finally:
            self._cleanup()

    def _init_hardware(self) -> bool:
        """Initialize all hardware."""
        logger.info("Initializing hardware...")

        # Connect motor controller
        if not self.motor.connect():
            logger.error("Failed to connect to motor controller")
            return False

        # Start LIDAR
        if not self.lidar.start():
            logger.error("Failed to start LIDAR")
            return False

        # Start camera
        if not self.camera.start():
            logger.error("Failed to start camera")
            return False

        logger.info("Hardware initialized")
        return True

    def _cleanup(self):
        """Cleanup on shutdown."""
        logger.info("Cleaning up...")

        self._running = False

        # Stop motors first
        if self.motor.is_connected:
            self.motor.stop()
            self.motor.disconnect()

        # Stop sensors
        if self.lidar.is_running:
            self.lidar.stop()
        if self.camera.is_running:
            self.camera.stop()

        logger.info("Cleanup complete")

    def _shutdown(self):
        """Handle shutdown signal."""
        logger.info("Shutdown requested")
        self._running = False

    async def _control_loop(self):
        """Main control loop."""
        period = 1.0 / CONTROL_LOOP_HZ

        while self._running:
            loop_start = asyncio.get_event_loop().time()

            # 1. Update motor encoder reading
            self.motor.update()

            # 2. Get fused perception
            world_state = self.fusion.update()

            # Store for web access
            self._latest_world = world_state
            self._latest_scan = self.lidar.get_scan()
            self._latest_blobs = self.camera.get_blobs()

            # 3. Update track map (learning during lap 1)
            self.track_map.update(world_state)

            # 4. Get decision
            speed, steering = self.state_machine.decide(world_state, self.track_map)

            # 5. Execute
            self.motor.drive(speed, steering)

            # Check if done
            if self.state_machine.state == RobotState.DONE:
                logger.info("Race complete, stopping")
                self._running = False
                break

            # Maintain loop rate
            self._loop_count += 1
            elapsed = asyncio.get_event_loop().time() - loop_start
            sleep_time = max(0, period - elapsed)
            await asyncio.sleep(sleep_time)

            # Log stats periodically
            if self._loop_count % (CONTROL_LOOP_HZ * 5) == 0:  # Every 5 seconds
                self._log_stats(world_state)

    def _log_stats(self, world):
        """Log periodic statistics."""
        logger.info(
            f"Loop {self._loop_count}: "
            f"State={self.state_machine.state.name}, "
            f"Lap={self.state_machine.lap_count}, "
            f"Encoder={world.encoder_pos}, "
            f"Front={world.walls.front_distance:.0f}mm"
            if world.walls.front_distance
            else f"Front=None"
        )


async def main():
    """Entry point."""
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
    )

    controller = Controller()
    await controller.run()


if __name__ == "__main__":
    asyncio.run(main())
