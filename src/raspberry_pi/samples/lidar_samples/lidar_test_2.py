from pyrplidar import PyRPlidar
import time
import math
import os

lidar = PyRPlidar()
lidar.connect(port="/dev/ttyUSB0", baudrate=460800)
lidar.set_motor_pwm(660)
time.sleep(2)

# Radar settings
RADAR_WIDTH = 61   # Width (horizontal)
RADAR_HEIGHT = 31  # Height (half circle)
MAX_DIST = 1000    # mm (1m for testing)
ANGLE_OFFSET = 0  # Triangle = LIDAR 0°/360°

def clear_screen():
    os.system('clear')

def lidar_to_real_angle(lidar_angle):
    """Convert LIDAR angle to real angle (triangle = front = 0°)"""
    real = (lidar_angle + ANGLE_OFFSET) % 360
    # Convert to -180 to +180
    if real > 180:
        real = real - 360
    return real

def is_front_angle(real_angle):
    """Check if real angle is in front 180° (-90° to +90°)"""
    return -90 <= real_angle <= 90

def create_radar(points):
    grid = [[' ' for _ in range(RADAR_WIDTH)] for _ in range(RADAR_HEIGHT)]
    center_x = RADAR_WIDTH // 2
    center_y = RADAR_HEIGHT - 1

    # Draw distance arcs
    for ring_dist in [250, 500, 750]:
        r = (ring_dist / MAX_DIST) * (RADAR_HEIGHT - 2)
        for angle in range(-90, 91, 3):
            rad = math.radians(angle - 90)
            x = int(center_x + r * math.cos(rad) * 2)
            y = int(center_y + r * math.sin(rad))
            if 0 <= x < RADAR_WIDTH and 0 <= y < RADAR_HEIGHT:
                if grid[y][x] == ' ':
                    grid[y][x] = '·'

    # Draw center line (0°)
    for i in range(RADAR_HEIGHT):
        if grid[i][center_x] == ' ':
            grid[i][center_x] = '│'

    # Draw base line
    for i in range(RADAR_WIDTH):
        if grid[RADAR_HEIGHT-1][i] == ' ':
            grid[RADAR_HEIGHT-1][i] = '─'
    grid[RADAR_HEIGHT-1][center_x] = '┴'

    # Add labels
    grid[0][center_x] = '0'
    grid[0][center_x+1] = '°'

    grid[RADAR_HEIGHT-1][0] = '-'
    grid[RADAR_HEIGHT-1][1] = '9'
    grid[RADAR_HEIGHT-1][2] = '0'
    grid[RADAR_HEIGHT-1][3] = '°'

    grid[RADAR_HEIGHT-1][RADAR_WIDTH-4] = '+'
    grid[RADAR_HEIGHT-1][RADAR_WIDTH-3] = '9'
    grid[RADAR_HEIGHT-1][RADAR_WIDTH-2] = '0'
    grid[RADAR_HEIGHT-1][RADAR_WIDTH-1] = '°'

    # Plot points using polar coordinates
    # Origin at bottom center, 0° = up (front), +90° = right, -90° = left
    for lidar_angle, dist in points.items():
        real_angle = lidar_to_real_angle(lidar_angle)

        if dist > 0 and dist < MAX_DIST and is_front_angle(real_angle):
            # Convert to radians (0° = up, so use sin for x, cos for y)
            rad = math.radians(real_angle)
            r = (dist / MAX_DIST) * (RADAR_HEIGHT - 2)

            # Polar to cartesian: x = r*sin(θ), y = r*cos(θ)
            # sin(0)=0, cos(0)=1, so front (0°) goes straight up
            x = int(center_x + r * math.sin(rad) * 2)  # *2 for character aspect ratio
            y = int(center_y - r * math.cos(rad))       # minus because screen y is inverted

            if 0 <= x < RADAR_WIDTH and 0 <= y < RADAR_HEIGHT:
                grid[y][x] = '*'

    return grid

def print_radar(grid, closest, num_points):
    clear_screen()
    print("  RPLIDAR C1 - Front 180° (-90° to +90°)")
    print("=" * (RADAR_WIDTH + 2))
    for row in grid:
        print('|' + ''.join(row) + '|')
    print("=" * (RADAR_WIDTH + 2))
    print("Arcs: 25cm · 50cm · 75cm    Max: 1m    * = object")
    if closest[1] > 0:
        # Debug: show calculated x,y for closest point
        ang = closest[0]
        dist = closest[1]
        rad = math.radians(ang)
        r = (dist / MAX_DIST) * (RADAR_HEIGHT - 2)
        cx = RADAR_WIDTH // 2
        cy = RADAR_HEIGHT - 1
        calc_x = cx + r * math.sin(rad) * 2
        calc_y = cy - r * math.cos(rad)
        print(f"Closest: {dist:.0f}mm at {ang:+.0f}° → x={calc_x:.1f} y={calc_y:.1f} r={r:.1f}")
    print("Ctrl+C to stop")

print("Scanning... triangle (▲) = front")

try:
    scan_generator = lidar.start_scan()
    points = {}  # angle -> list of (distance, quality)
    scan_count = 0

    for scan in scan_generator():
        angle_key = int(scan.angle) % 360
        dist = scan.distance
        quality = scan.quality

        # Store all valid readings (ignore <60mm = robot body)
        if dist > 60:
            if angle_key not in points:
                points[angle_key] = []
            points[angle_key].append(dist)
        scan_count += 1

        if scan_count >= 360:
            # Average the readings for each angle
            averaged_points = {}
            for angle_key, dist_list in points.items():
                if dist_list:
                    averaged_points[angle_key] = sum(dist_list) / len(dist_list)

            # Find closest in front 180°
            valid = []
            for lidar_angle, dist in averaged_points.items():
                real_angle = lidar_to_real_angle(lidar_angle)
                if dist > 0 and is_front_angle(real_angle):
                    valid.append((real_angle, dist))

            closest = min(valid, key=lambda x: x[1]) if valid else (0, 0)

            grid = create_radar(averaged_points)
            print_radar(grid, closest, len(averaged_points))
            scan_count = 0
            points.clear()

except KeyboardInterrupt:
    print("\nStopping...")

finally:
    lidar.stop()
    lidar.set_motor_pwm(0)
    lidar.disconnect()
    print("Done")
