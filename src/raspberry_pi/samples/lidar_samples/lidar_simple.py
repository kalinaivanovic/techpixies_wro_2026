from pyrplidar import PyRPlidar
import time

lidar = PyRPlidar()
lidar.connect(port="/dev/ttyUSB0", baudrate=460800)
lidar.set_motor_pwm(660)
time.sleep(2)

print("FRONT DISTANCE MONITOR")
print("=" * 40)
print("Measuring... (Ctrl+C to stop)")

try:
    scan_generator = lidar.start_scan()
    front_readings = {}  # angle -> (distance, quality)
    scan_count = 0

    for scan in scan_generator():
        angle = int(scan.angle) % 360
        dist = scan.distance
        quality = scan.quality

        # Store reading
        front_readings[angle] = (dist, quality)
        scan_count += 1

        # Update display after full rotation
        if scan_count >= 360:
            # Get front angles: 350-360 and 0-10 (= -10° to +10°)
            front_angles = list(range(0, 11)) + list(range(350, 360))

            valid_distances = []
            for a in front_angles:
                if a in front_readings:
                    d, q = front_readings[a]
                    if d > 60 and q > 10:  # >60mm to ignore robot body
                        valid_distances.append(d)

            if valid_distances:
                avg_dist = sum(valid_distances) / len(valid_distances)
                min_dist = min(valid_distances)
                # Single line update (carriage return to overwrite)
                print(f"\rFront: {avg_dist:5.0f}mm ({avg_dist/10:.1f}cm)  Min: {min_dist:5.0f}mm  [{len(valid_distances)} readings]", end="", flush=True)
            else:
                print(f"\rFront: ---- no valid readings ----", end="", flush=True)

            scan_count = 0
            front_readings.clear()

except KeyboardInterrupt:
    print("\nStopping...")

finally:
    lidar.stop()
    lidar.set_motor_pwm(0)
    lidar.disconnect()
    print("Done")
