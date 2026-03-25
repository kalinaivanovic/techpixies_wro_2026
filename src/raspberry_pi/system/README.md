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

No external resistor needed - the script enables the Pi's internal pull-up resistor.

### How it works

```
        3.3V (internal to Pi chip)
         │
         ┊ pull-up resistor (built-in)
         │
GPIO 17 ─┤──── button ──── GND (Pin 9)
         │
     reads HIGH (1) normally
     reads LOW (0) when button pressed
```

Without the pull-up, the pin would "float" when the button is open — reading random noise. The internal pull-up holds it at a stable HIGH (3.3V). When the button is pressed, it connects the pin directly to GND, pulling it LOW (0V). The script detects this falling edge as a button press.

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

## After Updating the Service File

When `wro-button.service` or `button_daemon.py` changes (e.g. after `git pull`):

```bash
sudo cp system/wro-button.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl restart wro-button
```

## Wifi and SSH Access

- **Competition mode** disables wifi (`rfkill block wifi`). You will lose SSH access.
- **Stopping the robot** does NOT re-enable wifi. This is intentional to comply with WRO rules.
- To get wifi back, **double press** the button to start debug mode, which re-enables wifi.
- Or connect a monitor/keyboard and run `sudo rfkill unblock wifi` manually.

## Networking Setup

The Pi uses wifi for normal connectivity and ethernet as a fallback for direct cable connection.

### Wifi (primary)

Wifi stays on DHCP — it automatically gets an IP from whatever router it connects to.

To add a wifi network:

```bash
sudo nmcli device wifi connect "NetworkName" password "password"
```

Access the Pi via `raspberrypi.local` or check the router for the assigned IP.

### Ethernet (fallback, static IP)

Set a static IP on ethernet so you can always reach the Pi with a direct cable, even without a router.

#### On the Pi

```bash
# Check your ethernet connection name
nmcli connection show

# Set static IP (use the name from the output above, usually "Wired connection 1")
sudo nmcli connection modify "Wired connection 1" \
    ipv4.method manual \
    ipv4.addresses 192.168.10.1/24

# Apply the change
sudo nmcli connection up "Wired connection 1"

# Verify
ip addr show eth0
```

#### On your laptop

Set ethernet adapter to:

| Setting | Value |
|---|---|
| IP address | 192.168.10.2 |
| Subnet mask | 255.255.255.0 |
| Gateway | (leave empty) |

#### Usage

Plug an ethernet cable directly between the Pi and your laptop:

```bash
ssh kalina@192.168.10.1
```

Web interface: `http://192.168.10.1:8080`

#### To revert ethernet back to DHCP

```bash
sudo nmcli connection modify "Wired connection 1" ipv4.method auto
sudo nmcli connection up "Wired connection 1"
```

### Both at the same time

Wifi and ethernet can work simultaneously. Use wifi for internet and ethernet for direct access — no conflict.

## How It Works

1. systemd starts `button_daemon.py` on boot
2. Daemon waits for GPIO button press
3. On press, detects press type (short/double/long)
4. Launches `main.py` as a subprocess with appropriate flags
5. If robot crashes, daemon keeps running and waits for next press
6. If daemon crashes, systemd restarts it automatically (after 3s)
