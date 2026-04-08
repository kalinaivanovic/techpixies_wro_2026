#!/usr/bin/env python3
"""
Button daemon - watches GPIO button and launches robot in correct mode.

Single press:   Competition mode (no wifi, no web)
Long press:     Shutdown Raspberry Pi
Press while running: Stop robot program

Runs as systemd service, always watching.

GPIO wiring:
    Pin 11 (GPIO 17) ---- button ---- Pin 9 (GND)
"""

import subprocess
import signal
import sys
import time

import RPi.GPIO as GPIO

BUTTON_PIN = 17          # GPIO 17 = physical pin 11
LONG_PRESS_THRESHOLD = 5  # seconds to trigger shutdown
DEBOUNCE_MS = 200        # button debounce
PROJECT_DIR = "/home/kalina/techpixies_wro_2026/src/raspberry_pi"
PYTHON_BIN = f"{PROJECT_DIR}/bin/python"

robot_process = None  # subprocess handle


def wait_for_press():
    """Block until button is pressed."""
    GPIO.wait_for_edge(BUTTON_PIN, GPIO.FALLING)


def get_press_type():
    """Detect press type: 'short' or 'long'."""
    # Wait for button release to measure hold duration
    press_start = time.time()
    while GPIO.input(BUTTON_PIN) == GPIO.LOW:
        if time.time() - press_start > LONG_PRESS_THRESHOLD:
            return "long"
        time.sleep(0.05)
    return "short"


def start_robot():
    """Launch main.py as subprocess in competition mode."""
    global robot_process

    stop_robot()

    subprocess.run(["sudo", "rfkill", "block", "wifi"], check=False)
    cmd = [PYTHON_BIN, "main.py", "--motor", "--lidar"]

    print(f"Starting robot: {' '.join(cmd)}")
    robot_process = subprocess.Popen(cmd, cwd=PROJECT_DIR)


def stop_robot():
    """Stop running robot program gracefully."""
    global robot_process

    if robot_process and robot_process.poll() is None:
        print("Stopping robot...")
        robot_process.send_signal(signal.SIGINT)
        try:
            robot_process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            robot_process.kill()
        robot_process = None
        # Re-enable wifi so SSH access is restored
        subprocess.run(["sudo", "rfkill", "unblock", "wifi"], check=False)
        print("Wifi re-enabled")


def cleanup(signum=None, frame=None):
    """Clean shutdown on SIGTERM/SIGINT."""
    stop_robot()
    GPIO.cleanup()
    sys.exit(0)


def main():
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(BUTTON_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)

    signal.signal(signal.SIGTERM, cleanup)
    signal.signal(signal.SIGINT, cleanup)

    print("Button daemon started. Waiting for button...")

    while True:
        wait_for_press()
        time.sleep(DEBOUNCE_MS / 1000)

        # If robot is running, any press stops it
        if robot_process and robot_process.poll() is None:
            stop_robot()
            print("Robot stopped. Waiting for button...")
            time.sleep(1)  # avoid re-trigger
            continue

        press = get_press_type()

        if press == "long":
            print("Long press - shutting down Raspberry Pi!")
            stop_robot()
            GPIO.cleanup()
            subprocess.run(["sudo", "shutdown", "-h", "now"])
            sys.exit(0)

        else:
            start_robot()


if __name__ == "__main__":
    main()
