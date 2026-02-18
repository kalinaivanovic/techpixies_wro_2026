from pyrplidar import PyRPlidar
import time

lidar = PyRPlidar()
lidar.connect(port="/dev/ttyUSB0", baudrate=460800)
lidar.set_motor_pwm(660)
time.sleep(2)

MAX_DIST = 3000  # mm
BAR_WIDTH = 50

def clear_screen():
    print("\033[H\033[J", end="")

def make_bar(dist, max_dist=MAX_DIST):
    if dist <= 0:
        return "." * BAR_WIDTH + "  --"
    # Position marker based on distance (closer = more left)
    pos = int((dist / max_dist) * BAR_WIDTH)
    pos = min(pos, BAR_WIDTH - 1)
    bar = "." * pos + "*" + "." * (BAR_WIDTH - pos - 1)
    return f"{bar} {dist:4.0f}mm"

print("LIDAR - Front 180°")
print("Scanning...")

try:
    scan_generator = lidar.start_scan()
    points = {}
    scan_count = 0

    for scan in scan_generator():
        angle = int(scan.angle) % 360
        dist = scan.distance
        quality = scan.quality

        if dist > 60 and quality > 5:
            if angle not in points:
                points[angle] = []
            points[angle].append(dist)
        scan_count += 1

        if scan_count >= 360:
            clear_screen()
            print("LIDAR Distance - Front 180°")
            print("LIDAR here                                        3m away")
            print("|" + "-" * BAR_WIDTH + "|")

            # Average readings
            avg = {}
            for a, dists in points.items():
                if dists:
                    avg[a] = sum(dists) / len(dists)

            angles_to_show = [
                (-90, "L 90°"),
                (-60, "  60°"),
                (-45, "  45°"),
                (-30, "  30°"),
                (-15, "  15°"),
                (0,   "   0°"),
                (15,  "  15°"),
                (30,  "  30°"),
                (45,  "  45°"),
                (60,  "  60°"),
                (90,  "R 90°"),
            ]

            for real_angle, label in angles_to_show:
                lidar_angle = real_angle % 360
                dist = 0
                for offset in [0, -1, 1, -2, 2]:
                    check_angle = (lidar_angle + offset) % 360
                    if check_angle in avg:
                        dist = avg[check_angle]
                        break
                print(f"{label} {make_bar(dist)}")

            # Find closest
            front_dists = []
            for a, d in avg.items():
                real = a if a <= 180 else a - 360
                if -90 <= real <= 90:
                    front_dists.append((real, d))

            if front_dists:
                closest = min(front_dists, key=lambda x: x[1])
                print()
                print(f"CLOSEST: {closest[1]:.0f}mm at {closest[0]:+.0f}°")

            print()
            print("Ctrl+C to stop")

            scan_count = 0
            points.clear()

except KeyboardInterrupt:
    print("\nStopping...")

finally:
    lidar.stop()
    lidar.set_motor_pwm(0)
    lidar.disconnect()
    print("Done")
