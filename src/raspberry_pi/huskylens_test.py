"""
HuskyLens 2 Test Script for Raspberry Pi
Tests I2C connection and basic object recognition
"""

from pinpong.board import Board
import time
from dfrobot_huskylensv2 import (
    HuskylensV2_I2C,
    ALGORITHM_OBJECT_RECOGNITION,
    ALGORITHM_COLOR_RECOGNITION,
)

# Initialize Raspberry Pi board
Board("rpi").begin()

# Create HuskyLens I2C connection (address 0x50, bus 1)
huskylens = HuskylensV2_I2C(bus_num=1)

# Check connection
if huskylens.knock():
    print("HuskyLens 2 connected!")
else:
    print("HuskyLens 2 not found. Check wiring and I2C settings.")
    exit(1)

# Switch to color recognition (useful for WRO pillars)
huskylens.switchAlgorithm(ALGORITHM_COLOR_RECOGNITION)
print("Switched to COLOR_RECOGNITION mode")
print("Press Ctrl+C to exit\n")

try:
    while True:
        huskylens.getResult(ALGORITHM_COLOR_RECOGNITION)

        if huskylens.available(ALGORITHM_COLOR_RECOGNITION):
            result = huskylens.getCachedCenterResult(ALGORITHM_COLOR_RECOGNITION)
            if result:
                print(f"ID: {result.ID}")
                print(f"Center: ({result.xCenter}, {result.yCenter})")
                print(f"Size: {result.width} x {result.height}")
                print("-" * 30)

        time.sleep(0.1)

except KeyboardInterrupt:
    print("\nExiting...")
