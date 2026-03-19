# System - Button Daemon

Standalone daemon that watches a physical GPIO button and launches the robot program in the correct mode.

## Button Actions

| Action | Result |
|---|---|
| Single press | Competition mode (wifi off, no web server) |
| Double press (within 5s) | Debug mode (wifi on, web server enabled) |
| Long press (hold > 5s) | Shutdown Raspberry Pi |
| Press while robot running | Stop robot program |

## Wiring

```
Pin 11 (GPIO 17) ---- button ---- Pin 9 (GND)
```

No external resistor needed - the script enables the Pi's internal pull-up.

## Installation

### 1. Allow passwordless sudo for rfkill and shutdown

Create `/etc/sudoers.d/wro`:

```bash
sudo visudo -f /etc/sudoers.d/wro
```

Add this line:

```
kalina ALL=(ALL) NOPASSWD: /usr/sbin/rfkill, /sbin/shutdown
```

### 2. Install the systemd service

```bash
sudo cp system/wro-button.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable wro-button
sudo systemctl start wro-button
```

### 3. Verify it's running

```bash
sudo systemctl status wro-button
```

## Useful Commands

```bash
# View live logs
journalctl -u wro-button -f

# Restart the daemon
sudo systemctl restart wro-button

# Stop the daemon
sudo systemctl stop wro-button

# Disable auto-start on boot
sudo systemctl disable wro-button
```

## How It Works

1. systemd starts `button_daemon.py` on boot
2. Daemon waits for GPIO button press
3. On press, detects press type (short/double/long)
4. Launches `main.py` as a subprocess with appropriate flags
5. If robot crashes, daemon keeps running and waits for next press
6. If daemon crashes, systemd restarts it automatically (after 3s)
