"""
Configuration constants for WRO 2025 robot.

All tunable parameters in one place.
"""

# =============================================================================
# HARDWARE PORTS
# =============================================================================

# LIDAR (RPLIDAR C1)
LIDAR_PORT = "/dev/ttyUSB0"
LIDAR_BAUDRATE = 460800

# ESP32 (Motor controller)
ESP32_PORT = "/dev/ttyUSB1"
ESP32_BAUDRATE = 115200

# Camera
CAMERA_INDEX = 0
CAMERA_WIDTH = 640
CAMERA_HEIGHT = 480
CAMERA_FOV = 120  # degrees

# =============================================================================
# ROBOT GEOMETRY (mm)
# =============================================================================

ROBOT_LENGTH = 200
ROBOT_WIDTH = 150
WHEELBASE = 160  # Front to rear axle
LIDAR_OFFSET_X = 50  # LIDAR distance from robot center (forward)

# =============================================================================
# TRACK CONSTANTS (from WRO rules)
# =============================================================================

TRACK_SIZE = 3000  # 3m x 3m
WALL_HEIGHT = 100
CORRIDOR_MIN = 600
CORRIDOR_MAX = 1000
PILLAR_SIZE = 50  # 50x50x100mm pillars

# =============================================================================
# CONTROL PARAMETERS
# =============================================================================

CONTROL_LOOP_HZ = 50
NORMAL_SPEED = 60  # -100 to 100
SLOW_SPEED = 35
STEERING_CENTER = 90  # Servo center position (0-180)

# Wall following
WALL_FOLLOW_KP = 0.5  # Proportional gain
MIN_WALL_CLEARANCE = 150  # mm

# Corner detection
CORNER_THRESHOLD = 400  # mm - front wall closer than this = corner

# Pillar avoidance
PILLAR_CLEARANCE = 100  # mm extra space around pillars

# =============================================================================
# LIDAR PROCESSING
# =============================================================================

LIDAR_MIN_DISTANCE = 60  # Ignore readings closer than this (robot body)
LIDAR_MAX_DISTANCE = 3000  # Ignore readings further than this
LIDAR_MIN_QUALITY = 10  # Minimum quality to accept reading

# Clustering
CLUSTER_ANGLE_GAP = 5  # Max degrees between points in same cluster
CLUSTER_DISTANCE_DIFF = 150  # Max mm difference in distance for same cluster
CLUSTER_MIN_POINTS = 3  # Minimum points to form a cluster

# =============================================================================
# CAMERA / COLOR DETECTION (HSV ranges for OpenCV)
# =============================================================================

# Red (wraps around hue, needs two ranges)
RED_LOWER1 = (0, 100, 100)
RED_UPPER1 = (10, 255, 255)
RED_LOWER2 = (160, 100, 100)
RED_UPPER2 = (180, 255, 255)

# Green
GREEN_LOWER = (40, 50, 50)
GREEN_UPPER = (80, 255, 255)

# Magenta (parking markers)
MAGENTA_LOWER = (140, 100, 100)
MAGENTA_UPPER = (160, 255, 255)

# Minimum contour area to detect (pixels)
MIN_CONTOUR_AREA = 300

# =============================================================================
# SENSOR FUSION
# =============================================================================

# Angle matching threshold for LIDAR-camera fusion (degrees)
ANGLE_MATCH_THRESHOLD = 40.0

# Expected pillar size range (mm)
PILLAR_SIZE_MIN = 30.0
PILLAR_SIZE_MAX = 1000.0

# =============================================================================
# WEB INTERFACE
# =============================================================================

WEB_HOST = "0.0.0.0"
WEB_PORT = 8080
