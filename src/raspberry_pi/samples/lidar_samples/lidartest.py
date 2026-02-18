from pyrplidar import PyRPlidar
import time

lidar = PyRPlidar()
lidar.connect(port="/dev/ttyUSB0",baudrate=460800)
time.sleep(2)
#Start scanning
lidar.set_motor_pwm(660)
time.sleep(2)

try:
    #Get device info:
    info = lidar.get_info()
    print(f"Model: {info.model}, Firmware: {info.firmware_major}.{info.firmware_minor}")

    #Get health
    health = lidar.get_health()
    print(f"Health: {health}")

    print("Scanning.... Press Ctrl+C to stop")
    scan_generator = lidar.start_scan()
    for i,scan in enumerate(scan_generator()):
        quality = scan.quality
        angle   = scan.angle
        distance = scan.distance
        if distance > 0: #This is valid reading
            print(f"Angle: {angle:6.1f}°  Distance: {distance:6.0f}mm Quality: {quality}")

except KeyboardInterrupt: 
    print ("\n Stopping...")
except Exception as e:
    print(e)
finally:
    lidar.stop()
    lidar.set_motor_pwm(0)
    lidar.disconnect()
    print("Done")