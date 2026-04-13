#!/usr/bin/env python3
"""
Button daemon - watches GPIO button and controls robot via web API.

The web backend (main.py --web --motor --lidar) runs continuously so
sensors stay warm. The button just toggles autonomous mode via REST.

On startup:          Launch web backend, wait until ready
Single press:        Block wifi, POST /api/auto/start
Press while running: POST /api/auto/stop, unblock wifi
Long press (5s):     Stop robot, stop backend, shutdown Pi

Curl targets localhost, so blocking wifi doesn't affect control.

GPIO wiring:
    Pin 11 (GPIO 17) ---- button ---- Pin 9 (GND)
"""

import subprocess
import signal
import sys
import time
import urllib.request
import urllib.error

import RPi.GPIO as GPIO

BUTTON_PIN = 17
LONG_PRESS_THRESHOLD = 5  # seconds
DEBOUNCE_MS = 200
PROJECT_DIR = "/home/kalina/techpixies_wro_2026/src/raspberry_pi"
PYTHON_BIN = f"{PROJECT_DIR}/bin/python"
WEB_URL = "http://localhost:8080"
WEB_READY_TIMEOUT = 60

web_process = None
auto_running = False


def wait_for_press():
    """Block until button is pressed."""
    GPIO.wait_for_edge(BUTTON_PIN, GPIO.FALLING)


def is_long_press():
    """Wait for release. Return True if held longer than LONG_PRESS_THRESHOLD."""
    press_start = time.time()
    while GPIO.input(BUTTON_PIN) == GPIO.LOW:
        if time.time() - press_start > LONG_PRESS_THRESHOLD:
            return True
        time.sleep(0.05)
    return False


def http(path, method="POST"):
    """Call local REST API. Returns (status, body) or (None, error)."""
    req = urllib.request.Request(f"{WEB_URL}{path}", method=method)
    try:
        with urllib.request.urlopen(req, timeout=5) as resp:
            return resp.status, resp.read().decode(errors="ignore")
    except urllib.error.URLError as e:
        return None, str(e)
    except Exception as e:
        return None, str(e)


def start_web_backend():
    """Launch main.py --web --motor --lidar and block until /api/status responds."""
    global web_process
    if web_process and web_process.poll() is None:
        return True

    cmd = [PYTHON_BIN, "main.py", "--web", "--motor", "--lidar"]
    print(f"Starting web backend: {' '.join(cmd)}")
    web_process = subprocess.Popen(cmd, cwd=PROJECT_DIR)

    deadline = time.time() + WEB_READY_TIMEOUT
    while time.time() < deadline:
        if web_process.poll() is not None:
            print("Web backend exited during startup")
            return False
        status, _ = http("/api/status", method="GET")
        if status == 200:
            print("Web backend ready")
            return True
        time.sleep(0.5)

    print("Web backend did not become ready in time")
    return False


def stop_web_backend():
    global web_process
    if web_process and web_process.poll() is None:
        print("Stopping web backend...")
        web_process.send_signal(signal.SIGINT)
        try:
            web_process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            web_process.kill()
    web_process = None


def start_auto():
    """Block wifi and start autonomous mode via REST."""
    global auto_running
    subprocess.run(["sudo", "rfkill", "block", "wifi"], check=False)
    status, body = http("/api/auto/start")
    if status == 200:
        auto_running = True
        print(f"Auto started: {body}")
    else:
        print(f"Failed to start auto (status={status}): {body}")
        subprocess.run(["sudo", "rfkill", "unblock", "wifi"], check=False)


def stop_auto():
    """Stop autonomous mode and re-enable wifi."""
    global auto_running
    if auto_running:
        status, body = http("/api/auto/stop")
        print(f"Auto stopped (status={status}): {body}")
    auto_running = False
    subprocess.run(["sudo", "rfkill", "unblock", "wifi"], check=False)
    print("Wifi re-enabled")


def cleanup(signum=None, frame=None):
    """Clean shutdown on SIGTERM/SIGINT."""
    stop_auto()
    stop_web_backend()
    GPIO.cleanup()
    sys.exit(0)


def main():
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(BUTTON_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)

    signal.signal(signal.SIGTERM, cleanup)
    signal.signal(signal.SIGINT, cleanup)

    # Wifi on while idle — phone can tune parameters via web UI
    subprocess.run(["sudo", "rfkill", "unblock", "wifi"], check=False)

    if not start_web_backend():
        print("Failed to start web backend, exiting")
        sys.exit(1)

    print("Button daemon ready. Waiting for button...")

    while True:
        wait_for_press()
        time.sleep(DEBOUNCE_MS / 1000)

        if auto_running:
            stop_auto()
            print("Auto stopped. Waiting for button...")
            time.sleep(1)
            continue

        if is_long_press():
            print("Long press - shutting down Raspberry Pi!")
            stop_auto()
            stop_web_backend()
            GPIO.cleanup()
            subprocess.run(["sudo", "shutdown", "-h", "now"])
            sys.exit(0)

        start_auto()


if __name__ == "__main__":
    main()
