from pyrplidar import PyRPlidar
import time
import math
import curses

# Radar settings
RADAR_WIDTH = 61
RADAR_HEIGHT = 27
RETENTION_FRAMES = 5

# Clustering settings
CLUSTER_DIST_THRESHOLD = 150  # mm - max distance difference to be same object
CLUSTER_MIN_POINTS = 3        # minimum points to form an object

# Adjustable settings with min/max
settings = {
    'dist': {'val': 2000, 'min': 500, 'max': 12000, 'step': 500, 'label': 'Distance'},
    'angle': {'val': 90, 'min': 15, 'max': 180, 'step': 15, 'label': 'Angle ±'},
    'quality': {'val': 0, 'min': 0, 'max': 50, 'step': 5, 'label': 'Min Qual'},
}
selected_setting = 0

# Object tracking
tracked_objects = {}  # id -> {'centroid': (angle, dist), 'velocity': float, 'age': int}
next_object_id = 0

def lidar_to_real_angle(lidar_angle, offset=0):
    real = (lidar_angle + offset) % 360
    if real > 180:
        real = real - 360
    return real

def is_front_angle(real_angle, angle_limit):
    return -angle_limit <= real_angle <= angle_limit

def get_arc_distances(max_d):
    if max_d <= 1000:
        return [250, 500, 750]
    elif max_d <= 2000:
        return [500, 1000, 1500]
    elif max_d <= 4000:
        return [1000, 2000, 3000]
    else:
        return [2000, 4000, 6000]

def cluster_points(points, angle_range, min_quality):
    """Group points into clusters (objects) based on distance continuity"""
    # Sort points by angle
    sorted_points = []
    for lidar_angle, (dist, quality) in points.items():
        real_angle = lidar_to_real_angle(lidar_angle)
        if quality >= min_quality and dist > 0 and is_front_angle(real_angle, angle_range):
            sorted_points.append((real_angle, dist))

    sorted_points.sort(key=lambda x: x[0])

    if not sorted_points:
        return []

    # Cluster consecutive points with similar distances
    clusters = []
    current_cluster = [sorted_points[0]]

    for i in range(1, len(sorted_points)):
        prev_angle, prev_dist = sorted_points[i-1]
        curr_angle, curr_dist = sorted_points[i]

        # Check if this point belongs to current cluster
        angle_gap = abs(curr_angle - prev_angle)
        dist_diff = abs(curr_dist - prev_dist)

        if angle_gap <= 5 and dist_diff < CLUSTER_DIST_THRESHOLD:
            current_cluster.append(sorted_points[i])
        else:
            # Save current cluster if large enough
            if len(current_cluster) >= CLUSTER_MIN_POINTS:
                clusters.append(current_cluster)
            current_cluster = [sorted_points[i]]

    # Don't forget last cluster
    if len(current_cluster) >= CLUSTER_MIN_POINTS:
        clusters.append(current_cluster)

    return clusters

def calculate_cluster_properties(cluster):
    """Calculate centroid, size, and bounding box of a cluster"""
    angles = [p[0] for p in cluster]
    dists = [p[1] for p in cluster]

    centroid_angle = sum(angles) / len(angles)
    centroid_dist = sum(dists) / len(dists)

    min_angle = min(angles)
    max_angle = max(angles)
    min_dist = min(dists)
    max_dist = max(dists)

    angular_width = max_angle - min_angle
    depth = max_dist - min_dist

    # Estimate physical width at centroid distance (arc length)
    physical_width = centroid_dist * math.radians(angular_width)

    return {
        'centroid': (centroid_angle, centroid_dist),
        'min_angle': min_angle,
        'max_angle': max_angle,
        'min_dist': min_dist,
        'max_dist': max_dist,
        'angular_width': angular_width,
        'physical_width': physical_width,
        'depth': depth,
        'num_points': len(cluster)
    }

def match_and_track_objects(clusters, dt=0.1):
    """Match current clusters to tracked objects and calculate velocities"""
    global tracked_objects, next_object_id

    cluster_props = [calculate_cluster_properties(c) for c in clusters]

    # Simple matching: closest centroid within threshold
    matched = set()
    new_tracked = {}

    for props in cluster_props:
        best_match = None
        best_distance = float('inf')

        for obj_id, obj in tracked_objects.items():
            if obj_id in matched:
                continue

            # Calculate distance between centroids
            angle_diff = abs(props['centroid'][0] - obj['centroid'][0])
            dist_diff = abs(props['centroid'][1] - obj['centroid'][1])

            # Combined distance metric
            match_dist = math.sqrt(angle_diff**2 + (dist_diff/100)**2)

            if match_dist < 20 and match_dist < best_distance:  # threshold for matching
                best_match = obj_id
                best_distance = match_dist

        if best_match is not None:
            # Update existing object
            old_dist = tracked_objects[best_match]['centroid'][1]
            new_dist = props['centroid'][1]

            # Get distance history, add new reading
            dist_history = tracked_objects[best_match].get('dist_history', [old_dist])
            dist_history.append(new_dist)
            if len(dist_history) > 5:  # Keep last 5 readings
                dist_history.pop(0)

            # Calculate velocity from history (more stable)
            if len(dist_history) >= 2:
                total_change = dist_history[0] - dist_history[-1]
                total_time = dt * (len(dist_history) - 1)
                velocity = total_change / total_time if total_time > 0 else 0
            else:
                velocity = 0

            # Heavy smoothing with previous velocity
            old_vel = tracked_objects[best_match].get('velocity', 0)
            smoothed_vel = 0.3 * velocity + 0.7 * old_vel

            # Deadband: ignore very small velocities (noise)
            if abs(smoothed_vel) < 20:
                smoothed_vel = 0

            new_tracked[best_match] = {
                'centroid': props['centroid'],
                'props': props,
                'velocity': smoothed_vel,
                'dist_history': dist_history,
                'age': 0
            }
            matched.add(best_match)
        else:
            # New object
            new_tracked[next_object_id] = {
                'centroid': props['centroid'],
                'props': props,
                'velocity': 0,
                'dist_history': [props['centroid'][1]],
                'age': 0
            }
            next_object_id += 1

    # Age unmatched objects and keep them briefly
    for obj_id, obj in tracked_objects.items():
        if obj_id not in matched:
            obj['age'] += 1
            if obj['age'] < 3:  # Keep for 3 frames
                new_tracked[obj_id] = obj

    tracked_objects = new_tracked
    return tracked_objects

def polar_to_screen(angle, dist, max_dist, center_x, center_y, height):
    """Convert polar coordinates to screen coordinates"""
    rad = math.radians(angle)
    r = (dist / max_dist) * (height - 2)
    x = int(center_x + r * math.sin(rad) * 2)
    y = int(center_y - r * math.cos(rad))
    return x, y

def draw_slider(stdscr, y, x, width, setting, is_selected):
    s = settings[setting]
    label = s['label']
    val = s['val']
    min_v = s['min']
    max_v = s['max']

    pct = (val - min_v) / (max_v - min_v)
    fill_width = int(pct * (width - 2))

    if setting == 'dist':
        val_str = f"{val/1000:.1f}m"
    elif setting == 'angle':
        val_str = f"±{val}°"
    else:
        val_str = f"{val}"

    attr = curses.A_BOLD if is_selected else 0
    color = curses.color_pair(6) if is_selected else curses.color_pair(5)

    stdscr.addstr(y, x, f"{label:10}", attr | color)
    stdscr.addstr(y, x + 11, "[")
    stdscr.addstr(y, x + 12, "=" * fill_width, curses.color_pair(1))
    stdscr.addstr(y, x + 12 + fill_width, " " * (width - 2 - fill_width))
    stdscr.addstr(y, x + 10 + width, "]")
    stdscr.addstr(y, x + 12 + width, f" {val_str:>8}", attr)

    if is_selected:
        stdscr.addstr(y, x - 2, "►", curses.color_pair(6))
    else:
        stdscr.addstr(y, x - 2, " ")

def create_radar_with_objects(objects, max_d, angle_limit):
    """Create radar grid with object rectangles"""
    grid = [[(' ', 0) for _ in range(RADAR_WIDTH)] for _ in range(RADAR_HEIGHT)]
    center_x = RADAR_WIDTH // 2
    center_y = RADAR_HEIGHT - 1

    # Draw distance arcs
    for ring_dist in get_arc_distances(max_d):
        if ring_dist < max_d:
            r = (ring_dist / max_d) * (RADAR_HEIGHT - 2)
            for angle in range(-angle_limit, angle_limit + 1, 3):
                rad = math.radians(angle)
                x = int(center_x + r * math.sin(rad) * 2)
                y = int(center_y - r * math.cos(rad))
                if 0 <= x < RADAR_WIDTH and 0 <= y < RADAR_HEIGHT:
                    if grid[y][x][0] == ' ':
                        grid[y][x] = ('·', 5)

    # Draw center line
    for i in range(RADAR_HEIGHT):
        if grid[i][center_x][0] == ' ':
            grid[i][center_x] = ('|', 5)

    # Draw base line
    for i in range(RADAR_WIDTH):
        if grid[RADAR_HEIGHT-1][i][0] == ' ':
            grid[RADAR_HEIGHT-1][i] = ('-', 5)
    grid[RADAR_HEIGHT-1][center_x] = ('+', 5)

    # Draw objects as rectangles
    for obj_id, obj in objects.items():
        props = obj.get('props')
        if not props:
            continue

        velocity = obj.get('velocity', 0)

        # Determine color based on velocity
        if velocity > 50:  # approaching fast
            color = 4  # red
        elif velocity > 10:  # approaching
            color = 3  # yellow
        elif velocity < -10:  # receding
            color = 2  # cyan
        else:  # stationary
            color = 1  # green

        # Get corner positions
        min_angle = props['min_angle']
        max_angle = props['max_angle']
        min_dist = props['min_dist']
        max_dist = min(props['max_dist'], max_d)

        if max_dist <= 0 or min_dist >= max_d:
            continue

        # Draw rectangle outline
        # Top edge (far)
        for ang in range(int(min_angle), int(max_angle) + 1):
            x, y = polar_to_screen(ang, max_dist, max_d, center_x, center_y, RADAR_HEIGHT)
            if 0 <= x < RADAR_WIDTH and 0 <= y < RADAR_HEIGHT:
                grid[y][x] = ('-', color)

        # Bottom edge (near)
        for ang in range(int(min_angle), int(max_angle) + 1):
            x, y = polar_to_screen(ang, min_dist, max_d, center_x, center_y, RADAR_HEIGHT)
            if 0 <= x < RADAR_WIDTH and 0 <= y < RADAR_HEIGHT:
                grid[y][x] = ('-', color)

        # Left edge
        for d in range(int(min_dist), int(max_dist) + 1, 50):
            x, y = polar_to_screen(min_angle, d, max_d, center_x, center_y, RADAR_HEIGHT)
            if 0 <= x < RADAR_WIDTH and 0 <= y < RADAR_HEIGHT:
                grid[y][x] = ('|', color)

        # Right edge
        for d in range(int(min_dist), int(max_dist) + 1, 50):
            x, y = polar_to_screen(max_angle, d, max_d, center_x, center_y, RADAR_HEIGHT)
            if 0 <= x < RADAR_WIDTH and 0 <= y < RADAR_HEIGHT:
                grid[y][x] = ('|', color)

        # Draw velocity at centroid
        cent_x, cent_y = polar_to_screen(props['centroid'][0], props['centroid'][1],
                                          max_d, center_x, center_y, RADAR_HEIGHT)

        # Format velocity string
        if abs(velocity) < 10:
            vel_str = "0"
        else:
            vel_str = f"{velocity/1000:.1f}" if abs(velocity) >= 100 else f"{velocity:.0f}"

        # Draw velocity (try to fit it)
        for i, char in enumerate(vel_str[:5]):
            px = cent_x - len(vel_str)//2 + i
            if 0 <= px < RADAR_WIDTH and 0 <= cent_y < RADAR_HEIGHT:
                grid[cent_y][px] = (char, color)

    return grid

def main(stdscr):
    global selected_setting, tracked_objects, next_object_id

    curses.curs_set(0)
    stdscr.nodelay(True)
    curses.start_color()

    curses.init_pair(1, curses.COLOR_GREEN, curses.COLOR_BLACK)
    curses.init_pair(2, curses.COLOR_CYAN, curses.COLOR_BLACK)
    curses.init_pair(3, curses.COLOR_YELLOW, curses.COLOR_BLACK)
    curses.init_pair(4, curses.COLOR_RED, curses.COLOR_BLACK)
    curses.init_pair(5, curses.COLOR_WHITE, curses.COLOR_BLACK)
    curses.init_pair(6, curses.COLOR_MAGENTA, curses.COLOR_BLACK)

    stdscr.clear()

    lidar = PyRPlidar()
    lidar.connect(port="/dev/ttyUSB0", baudrate=460800)
    lidar.set_motor_pwm(660)
    time.sleep(2)

    setting_keys = list(settings.keys())
    last_time = time.time()

    try:
        scan_generator = lidar.start_scan()
        points = {}
        scan_count = 0

        for scan in scan_generator():
            try:
                key = stdscr.getch()
                if key == ord('q'):
                    break
                elif key == curses.KEY_UP or key == ord('k'):
                    selected_setting = (selected_setting - 1) % len(setting_keys)
                elif key == curses.KEY_DOWN or key == ord('j') or key == ord('\t'):
                    selected_setting = (selected_setting + 1) % len(setting_keys)
                elif key == curses.KEY_RIGHT or key == ord('l') or key == ord('+') or key == ord('='):
                    s = settings[setting_keys[selected_setting]]
                    s['val'] = min(s['val'] + s['step'], s['max'])
                elif key == curses.KEY_LEFT or key == ord('h') or key == ord('-'):
                    s = settings[setting_keys[selected_setting]]
                    s['val'] = max(s['val'] - s['step'], s['min'])
            except:
                pass

            angle_key = int(scan.angle) % 360
            dist = scan.distance
            quality = scan.quality

            if dist > 60:
                if angle_key not in points:
                    points[angle_key] = []
                points[angle_key].append((dist, quality))
            scan_count += 1

            if scan_count >= 360:
                # Calculate time delta
                current_time = time.time()
                dt = current_time - last_time
                last_time = current_time

                # Average readings
                averaged_points = {}
                for angle_key, readings in points.items():
                    if readings:
                        avg_dist = sum(r[0] for r in readings) / len(readings)
                        avg_qual = sum(r[1] for r in readings) / len(readings)
                        averaged_points[angle_key] = (avg_dist, avg_qual)

                max_dist = settings['dist']['val']
                angle_range = settings['angle']['val']
                min_quality = settings['quality']['val']

                # Cluster points into objects
                clusters = cluster_points(averaged_points, angle_range, min_quality)

                # Track objects and calculate velocities
                objects = match_and_track_objects(clusters, dt)

                # Create radar with objects
                grid = create_radar_with_objects(objects, max_dist, angle_range)

                # Draw header
                stdscr.attron(curses.A_BOLD)
                stdscr.addstr(0, 0, "RPLIDAR C1 - Object Tracking")
                stdscr.attroff(curses.A_BOLD)
                stdscr.addstr(0, 30, "↑↓:select  ←→:adjust  q:quit")

                # Draw radar grid
                for row_idx, row in enumerate(grid):
                    for col_idx, (char, color) in enumerate(row):
                        try:
                            if color > 0:
                                stdscr.addstr(row_idx + 1, col_idx + 1, char, curses.color_pair(color))
                            else:
                                stdscr.addstr(row_idx + 1, col_idx + 1, char)
                        except curses.error:
                            pass
                    stdscr.addstr(row_idx + 1, 0, '|')
                    stdscr.addstr(row_idx + 1, RADAR_WIDTH + 1, '|')

                # Draw sliders
                slider_y = RADAR_HEIGHT + 2
                for i, key in enumerate(setting_keys):
                    draw_slider(stdscr, slider_y + i, 3, 30, key, i == selected_setting)

                # Draw legend
                info_y = slider_y + len(setting_keys) + 1
                stdscr.addstr(info_y, 0, "Velocity: ")
                stdscr.addstr("■", curses.color_pair(4))
                stdscr.addstr("→fast ")
                stdscr.addstr("■", curses.color_pair(3))
                stdscr.addstr("→slow ")
                stdscr.addstr("■", curses.color_pair(1))
                stdscr.addstr("still ")
                stdscr.addstr("■", curses.color_pair(2))
                stdscr.addstr("←away")

                # Show object count
                stdscr.addstr(info_y, 50, f"Objects: {len(objects)}   ")

                stdscr.refresh()

                scan_count = 0
                points.clear()

    finally:
        lidar.stop()
        lidar.set_motor_pwm(0)
        lidar.disconnect()

if __name__ == "__main__":
    curses.wrapper(main)
    print("Done")
