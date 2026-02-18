from pyrplidar import PyRPlidar
import time
import os

lidar = PyRPlidar()
lidar.connect(port="/dev/ttyUSB0", baudrate=460800)
lidar.set_motor_pwm(660)
time.sleep(2)

def clear_screen():
    os.system('clear')

print("Put obstacle in front of LIDAR and watch which angles detect it")
print("This helps find where 'front' actually is")
print()

try:
    scan_generator = lidar.start_scan()
    points = {}
    scan_count = 0

    for scan in scan_generator():
        angle = int(scan.angle) % 360
        dist = scan.distance
        points[angle] = dist
        scan_count += 1

        if scan_count >= 360:
            clear_screen()
            print("LIDAR Debug - Finding 'Front'")
            print("=" * 50)
            print("Put your hand/obstacle close (<500mm) and see which angles detect it")
            print()

            # Find all close objects (< 500mm)
            close_objects = [(a, d) for a, d in points.items() if 0 < d < 500]
            close_objects.sort(key=lambda x: x[0])

            if close_objects:
                print("CLOSE OBJECTS DETECTED (<500mm):")
                for angle, dist in close_objects:
                    bar = '█' * int(dist / 50)
                    print(f"  Angle {angle:3d}° : {dist:4.0f}mm {bar}")
                print()

                # Find center of close object
                avg_angle = sum(a for a, d in close_objects) / len(close_objects)
                min_dist = min(d for a, d in close_objects)
                print(f"  → Obstacle center: ~{avg_angle:.0f}°, closest: {min_dist:.0f}mm")
            else:
                print("No close objects detected. Move something closer.")

            print()
            print("-" * 50)
            print("Distance at key angles:")
            for angle in [0, 45, 90, 135, 180, 225, 270, 315]:
                d = points.get(angle, 0)
                if d > 0:
                    print(f"  {angle:3d}° : {d:5.0f}mm")
                else:
                    print(f"  {angle:3d}° : --")

            print()
            print("Ctrl+C to stop")
            scan_count = 0

except KeyboardInterrupt:
    print("\nStopping...")

finally:
    lidar.stop()
    lidar.set_motor_pwm(0)
    lidar.disconnect()
    print("Done")
