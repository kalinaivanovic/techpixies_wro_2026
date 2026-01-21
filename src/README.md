# Source Code

This directory contains all control software for the WRO 2025 Future Engineers robot.

## Structure

```
src/
├── esp32/              # ESP32-S3 motor controller (C++/Arduino)
│   ├── platformio.ini  # PlatformIO configuration
│   └── src/
│       ├── main.cpp    # Entry point
│       ├── motor.cpp/h # DC motor control with encoder
│       └── steering.cpp/h  # Servo steering control
│
└── raspberry_pi/       # Raspberry Pi brain (Python)
    ├── main.py         # Entry point
    ├── requirements.txt
    ├── drivers/        # Sensor drivers
    │   ├── lidar.py    # RPLIDAR C1 driver
    │   └── huskylens.py # HuskyLens AI camera
    ├── perception/     # Sensor fusion
    │   └── sensor_fusion.py
    ├── behavior/       # Reactive behaviors
    │   ├── wall_follow.py
    │   └── pillar_avoid.py
    ├── mission/        # High-level control
    │   └── state_machine.py
    └── comm/           # ESP32 communication
        └── esp32_serial.py
```

## Architecture

```
┌─────────────────────────────────────────┐
│         RASPBERRY PI (Python)           │
│                                         │
│  mission/     - State machine, laps     │
│  behavior/    - Wall follow, avoidance  │
│  perception/  - Sensor fusion           │
│  drivers/     - LIDAR, HuskyLens        │
│  comm/        - Serial to ESP32         │
└─────────────────┬───────────────────────┘
                  │ UART
┌─────────────────┴───────────────────────┐
│          ESP32-S3 (C++/Arduino)         │
│                                         │
│  - Receives speed/steering commands     │
│  - PID motor control with encoder       │
│  - Servo steering                       │
│  - Safety watchdog                      │
└─────────────────────────────────────────┘
```

## Building & Uploading

### ESP32-S3

```bash
cd src/esp32
pio run              # Build
pio run -t upload    # Upload to board
pio device monitor   # Serial monitor
```

### Raspberry Pi

```bash
cd src/raspberry_pi
pip install -r requirements.txt
python main.py
```

## Communication Protocol

The Raspberry Pi sends commands to ESP32 via serial (115200 baud):

```
Pi -> ESP32:  SPD:<speed>,STR:<angle>\n
ESP32 -> Pi:  OK,ENC:<count>\n
```

- `speed`: -100 to 100 (negative = reverse)
- `angle`: 0 to 180 (90 = center)
- `count`: encoder tick count
