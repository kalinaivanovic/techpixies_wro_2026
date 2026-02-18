from pyrplidar import PyRPlidar
import time
import math
import curses

# Radar settings
RADAR_WIDTH = 61
RADAR_HEIGHT = 27  # Reduced to make room for sliders
RETENTION_FRAMES = 5

# Adjustable settings with min/max
settings = {
    'dist': {'val': 1000, 'min': 500, 'max': 12000, 'step': 500, 'label': 'Distance'},
    'angle': {'val': 90, 'min': 15, 'max': 180, 'step': 15, 'label': 'Angle ±'},
    'quality': {'val': 0, 'min': 0, 'max': 50, 'step': 5, 'label': 'Min Qual'},
}
selected_setting = 0  # Which slider is selected

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

def draw_slider(stdscr, y, x, width, setting, is_selected):
    s = settings[setting]
    label = s['label']
    val = s['val']
    min_v = s['min']
    max_v = s['max']

    # Calculate fill percentage
    pct = (val - min_v) / (max_v - min_v)
    fill_width = int(pct * (width - 2))

    # Format value
    if setting == 'dist':
        val_str = f"{val/1000:.1f}m"
    elif setting == 'angle':
        val_str = f"±{val}°"
    else:
        val_str = f"{val}"

    # Draw label
    attr = curses.A_BOLD if is_selected else 0
    color = curses.color_pair(6) if is_selected else curses.color_pair(5)

    stdscr.addstr(y, x, f"{label:10}", attr | color)

    # Draw slider track
    stdscr.addstr(y, x + 11, "[")
    stdscr.addstr(y, x + 12, "=" * fill_width, curses.color_pair(1))
    stdscr.addstr(y, x + 12 + fill_width, " " * (width - 2 - fill_width))
    stdscr.addstr(y, x + 10 + width, "]")

    # Draw value
    stdscr.addstr(y, x + 12 + width, f" {val_str:>8}", attr)

    # Draw selection indicator
    if is_selected:
        stdscr.addstr(y, x - 2, "►", curses.color_pair(6))
    else:
        stdscr.addstr(y, x - 2, " ")

def create_radar(points, max_d, angle_limit, min_qual):
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

    # Plot points
    for lidar_angle, (dist, quality) in points.items():
        real_angle = lidar_to_real_angle(lidar_angle)
        if quality >= min_qual and dist > 0 and dist < max_d and is_front_angle(real_angle, angle_limit):
            rad = math.radians(real_angle)
            r = (dist / max_d) * (RADAR_HEIGHT - 2)
            x = int(center_x + r * math.sin(rad) * 2)
            y = int(center_y - r * math.cos(rad))
            if 0 <= x < RADAR_WIDTH and 0 <= y < RADAR_HEIGHT:
                if quality >= 40:
                    color = 1
                elif quality >= 25:
                    color = 2
                elif quality >= 15:
                    color = 3
                else:
                    color = 4
                grid[y][x] = ('*', color)

    return grid

def main(stdscr):
    global selected_setting

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

    try:
        scan_generator = lidar.start_scan()
        points = {}
        retained_points = {}
        scan_count = 0

        for scan in scan_generator():
            try:
                key = stdscr.getch()
                if key == ord('q'):
                    break
                # Navigate sliders with up/down or Tab
                elif key == curses.KEY_UP or key == ord('k'):
                    selected_setting = (selected_setting - 1) % len(setting_keys)
                elif key == curses.KEY_DOWN or key == ord('j') or key == ord('\t'):
                    selected_setting = (selected_setting + 1) % len(setting_keys)
                # Adjust values with left/right or h/l
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
                averaged_points = {}
                for angle_key, readings in points.items():
                    if readings:
                        avg_dist = sum(r[0] for r in readings) / len(readings)
                        avg_qual = sum(r[1] for r in readings) / len(readings)
                        averaged_points[angle_key] = (avg_dist, avg_qual)

                for angle_key, (d, q) in averaged_points.items():
                    retained_points[angle_key] = (d, q, 0)

                to_remove = []
                for angle_key, (d, q, age) in retained_points.items():
                    if angle_key not in averaged_points:
                        retained_points[angle_key] = (d, q, age + 1)
                    if retained_points[angle_key][2] >= RETENTION_FRAMES:
                        to_remove.append(angle_key)
                for angle_key in to_remove:
                    del retained_points[angle_key]

                display_points = {k: (v[0], v[1] * (1 - v[2]/RETENTION_FRAMES))
                                  for k, v in retained_points.items()}

                max_dist = settings['dist']['val']
                angle_range = settings['angle']['val']
                min_quality = settings['quality']['val']

                valid = []
                for lidar_angle, (d, q) in averaged_points.items():
                    real_angle = lidar_to_real_angle(lidar_angle)
                    if q >= min_quality and d > 0 and is_front_angle(real_angle, angle_range):
                        valid.append((real_angle, d, q))
                closest = min(valid, key=lambda x: x[1]) if valid else (0, 0, 0)

                grid = create_radar(display_points, max_dist, angle_range, min_quality)

                # Draw header
                stdscr.attron(curses.A_BOLD)
                stdscr.addstr(0, 0, "RPLIDAR C1")
                stdscr.attroff(curses.A_BOLD)
                stdscr.addstr(0, 12, "↑↓:select  ←→:adjust  q:quit")

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

                # Draw quality legend and closest
                info_y = slider_y + len(setting_keys) + 1
                stdscr.addstr(info_y, 0, "Quality: ")
                stdscr.addstr("●", curses.color_pair(1))
                stdscr.addstr(">40 ")
                stdscr.addstr("●", curses.color_pair(2))
                stdscr.addstr(">25 ")
                stdscr.addstr("●", curses.color_pair(3))
                stdscr.addstr(">15 ")
                stdscr.addstr("●", curses.color_pair(4))
                stdscr.addstr("<15")

                if closest[1] > 0:
                    stdscr.addstr(info_y, 45, "Closest: ")
                    stdscr.addstr(f"{closest[1]:.0f}mm", curses.color_pair(6) | curses.A_BOLD)
                    stdscr.addstr(f" at {closest[0]:+.0f}°   ")
                else:
                    stdscr.addstr(info_y, 45, "Closest: --              ")

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
