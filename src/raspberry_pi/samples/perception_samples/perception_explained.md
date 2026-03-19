# Perception Layer Explained

## Chapter 1: Black Box View

The perception layer answers one question: **"What does the robot see right now?"**

It takes raw sensor readings and produces a simple, clean description of the world.

### High-level diagram

```
                          ┌───────────────────────────────────┐
                          │                                   │
  dict[int, float]        │                                   │        WorldState
  LIDAR scan          ──► │                                   │ ──►    - walls (L/R/F distances)
  {angle: distance_mm}    │                                   │        - pillars (color + distance)
                          │                                   │        - corner_ahead
  list[ColorBlob]         │      PERCEPTION LAYER             │        - parking_marker
  Camera blobs        ──► │                                   │        - floor_line
  [color, angle,          │      SensorFusion.update()        │
   x, y, area]           │                                   │
                          │                                   │        TrackMap
  int                     │                                   │ ──►    - direction (CW/CCW)
  Encoder position    ──► │                                   │        - corners[]
  (ticks)                 │                                   │        - sections[] (widths)
                          │                                   │        - pillars[] (positions)
                          │                                   │        - lap counting
                          └───────────────────────────────────┘
```

### Inputs and outputs with example values

```
 INPUTS                                              OUTPUTS
 ======                                              =======

 LIDAR scan (360 degrees)
   {0: 2100, 1: 2095, ..., 90: 450, ..., 270: 380}
                                                     WorldState
 Camera blobs (colored regions)                        - walls: left=380mm, right=450mm, front=2100mm
   [red blob at 25deg, green blob at -18deg, ...]      - pillars: [RED at 25deg 310mm]
                                                       - corner_ahead: None
 Encoder position (how far we drove)                   - parking_marker: None
   4820                                                - floor_line: "orange"
```

### The domain objects

Here is every object that goes in and comes out, with example values from a real scenario.

#### Scenario

The robot is on a straight section, 380mm from the left wall, 450mm from the right wall.
A red pillar is ahead-right at 25 degrees, about 310mm away.
The robot just drove over an orange floor line.
No corner ahead, no parking marker visible.

---

#### INPUT: LIDAR scan

A dictionary. Key = angle in degrees (0-359), value = distance in mm.

Angle 0 is straight ahead. 90 is right. 270 is left.

```python
scan = {
    0: 2100,     # 2100mm to front wall (far away, no corner)
    1: 2095,
    2: 2080,
    # ... many angles ...
    25: 310,     # something close at 25 degrees (it's the red pillar)
    26: 305,
    27: 315,
    # ...
    89: 455,
    90: 450,     # right wall at 450mm
    91: 448,
    # ...
    269: 385,
    270: 380,    # left wall at 380mm
    271: 382,
    # ...
}
```

#### INPUT: Camera blobs

A list of colored regions detected in the camera frame.

```python
blobs = [
    ColorBlob(color="red",    angle=25.2,  x=387, y=210, width=32, height=55, area=1430),
    ColorBlob(color="orange", angle=-5.1,  x=303, y=410, width=120, height=25, area=2600),
]
```

Each blob has:
- `color` — what color the camera detected ("red", "green", "magenta", "orange", "blue")
- `angle` — degrees from center of frame (positive = right, negative = left)
- `x, y` — pixel coordinates in the camera frame
- `width, height, area` — size in pixels

The red blob is a pillar ahead-right.
The orange blob is a floor line near the bottom of the frame.

#### INPUT: Encoder position

A single integer. How many encoder ticks since the start. Increases as the robot drives forward.

```python
encoder = 4820
```

---

#### OUTPUT: WorldState

One object that summarizes everything the robot needs to know right now.

```python
WorldState(
    timestamp   = 1709654321.45,
    encoder_pos = 4820,

    walls = WallInfo(
        left_distance  = 380.0,   # mm to left wall
        right_distance = 450.0,   # mm to right wall
        front_distance = 2100.0,  # mm to front wall
    ),
    # corridor_width = 380 + 450 = 830mm (property, computed automatically)

    pillars = [
        Pillar(color="red", angle=25.2, distance=310.0),
        #  .pass_side     = "right"  (RED = pass on right)
        #  .is_blocking() = True     (25.2 < 35 degree threshold)
    ],

    corner_ahead    = None,       # no corner (front wall is 2100mm away)
    parking_marker  = None,       # no magenta blob visible
    floor_line      = "orange",   # orange line in bottom of camera frame
)
```

That's it. The decision layer receives this one object and decides what to do.

---

#### OUTPUT: TrackMap (accumulated over time)

While WorldState is one instant, TrackMap remembers everything from the whole run.
After driving half a lap, it might look like:

```python
TrackMap:
    direction      = "CW"
    corners        = [Corner(encoder_pos=1200, direction="RIGHT"),
                      Corner(encoder_pos=3600, direction="RIGHT")]
    sections       = [Section(start_encoder=0,    end_encoder=1200, width=830.0),
                      Section(start_encoder=1200, end_encoder=3600, width=620.0)]
    pillars        = [PillarRecord(encoder_pos=800,  color="red",   side="right", angle=25.0),
                      PillarRecord(encoder_pos=2400, color="green", side="left",  angle=-15.0)]
    parking_zone   = None
    lap_length     = None          # not yet known (still first lap)
    section_count  = 2             # crossed 2 orange/blue line pairs so far
    line_direction = "CW"          # first pair was orange then blue
```

---

## Chapter 2: Inside the Black Box

Now let's trace what happens inside `SensorFusion.update()` using the same example values.

### Step 1: Collect raw data

```python
scan    = self.lidar.get_scan()     # {0: 2100, ..., 25: 310, ..., 90: 450, ..., 270: 380, ...}
blobs   = self.camera.get_blobs()   # [ColorBlob(red, 25.2deg, ...), ColorBlob(orange, -5.1deg, ...)]
encoder = self.get_encoder()        # 4820
```

Three independent sensor reads. Each sensor runs in its own thread and we grab the latest value.

### Step 2: Wall detection

The wall detection strategy reads LIDAR distances at fixed angles.

Using `AverageWallDetection` (the simpler strategy):

```
Left wall:   average of scan[260]..scan[280]  →  ~380mm
Right wall:  average of scan[80]..scan[100]   →  ~450mm
Front wall:  average of scan[355]..scan[5]    →  ~2100mm
```

Result:
```python
walls = WallInfo(left_distance=380.0, right_distance=450.0, front_distance=2100.0)
```

With `ClusteringWallDetection` (the smarter strategy), clustering runs first to separate walls from pillars.
If the red pillar happens to be near 90 degrees, AverageWallDetection would mix pillar distance (310mm)
into the right wall average and give a wrong reading. ClusteringWallDetection avoids this because it only
considers objects classified as "wall" (width >= 120mm), not "pillar" (width < 120mm).

### Step 3: Corner detection

`LidarCornerDetection` checks how far the front wall is:

```
front distance = average of scan[355..5] = 2100mm
threshold = 400mm

2100 > 400  →  No corner
```

Result:
```python
corner_ahead = None
```

If the front distance were, say, 350mm, a corner would be detected. Then it would compare
left (270deg) vs right (90deg) to decide the direction:

```
If front = 350mm (< 400 threshold):
    left  = scan[270] = 380mm
    right = scan[90]  = 450mm
    right > left  →  corner_ahead = "RIGHT"
```

### Step 4: Object clustering

`OpenCVClustering` converts the LIDAR scan into an image and finds objects.

```
Step 4a: Draw LIDAR points as white pixels on a 500x500 black image (bird's eye view)

    Each point: angle + distance → (x, y) pixel

    scan[25] = 310mm → pixel (center + 310*sin(25deg)*scale, center - 310*cos(25deg)*scale)
                      → roughly pixel (271, 227)   (a dot ahead-right of center)

    scan[90] = 450mm → pixel (center + 450*sin(90deg)*scale, center - 450*cos(90deg)*scale)
                      → roughly pixel (287, 250)   (a dot to the right)

    ... hundreds of points drawn ...

Step 4b: Dilate the image (7x7 kernel, 2 iterations)

    Nearby white pixels merge into blobs. The wall becomes one long blob.
    The pillar's 3-4 nearby points become one small blob.

Step 4c: Find contours (cv2.findContours)

    Each contiguous white region becomes a contour.
    Result: ~3-5 contours (left wall, right wall, front wall, red pillar, maybe noise)

Step 4d: Classify each contour

    For each contour, measure its bounding box and convert back to mm:

    Contour 1 (left wall):   width_mm = 1800mm → "wall"    (>> 120mm threshold)
    Contour 2 (right wall):  width_mm = 1600mm → "wall"
    Contour 3 (front wall):  width_mm = 600mm  → "wall"
    Contour 4 (red pillar):  width_mm = 65mm   → "pillar"  (< 120mm threshold)
```

Result:
```python
objects = [
    DetectedObject(angle=270.0, distance=380.0,  width=1800.0, obj_type="wall"),
    DetectedObject(angle=90.0,  distance=450.0,  width=1600.0, obj_type="wall"),
    DetectedObject(angle=0.0,   distance=2100.0, width=600.0,  obj_type="wall"),
    DetectedObject(angle=25.5,  distance=310.0,  width=65.0,   obj_type="pillar"),
]
```

### Step 5: Pillar matching (the core of sensor fusion)

This is where LIDAR and camera data actually fuse together.

LIDAR knows: "there is a small object at 25.5 degrees, 310mm away"
Camera knows: "there is a red blob at 25.2 degrees"

Neither sensor alone is enough:
- LIDAR can't see color (doesn't know if pillar is red or green)
- Camera can't measure distance (just pixels)

The matching algorithm:

```
1. Filter camera blobs to red/green only:
   pillar_blobs = [ColorBlob(color="red", angle=25.2, ...)]
   (orange blob is filtered out — it's not a pillar color)

2. For the red blob (angle=25.2), search all LIDAR objects:

   Object 0: angle=270.0 (left wall)
     Camera uses signed angles: 0=forward, positive=right, negative=left
     LIDAR uses 0-360: 0=forward, 90=right, 270=left
     Convert: 270 → 270-360 = -90 (left in camera coords)
     Angle diff = |25.2 - (-90)| = 115.2     →  too far (> threshold)

   Object 1: angle=90.0 (right wall)
     Convert: 90 → 90 (right in camera coords)
     Angle diff = |25.2 - 90| = 64.8         →  too far

   Object 2: angle=0.0 (front wall)
     Convert: 0 → 0
     Angle diff = |25.2 - 0| = 25.2          →  too far

   Object 3: angle=25.5 (the pillar!)
     Convert: 25.5 → 25.5
     Angle diff = |25.2 - 25.5| = 0.3        →  MATCH! (< threshold)

3. Create confirmed pillar:
   Pillar(color="red", angle=25.2, distance=310.0)
         ^^^^^^^^^^^^               ^^^^^^^^^^^^
         from camera                from LIDAR
```

Result:
```python
pillars = [Pillar(color="red", angle=25.2, distance=310.0)]
```

What happens if there is NO match:
- A red blob in the camera but no LIDAR object nearby → ignored (could be a red t-shirt in audience)
- A LIDAR object but no colored blob → stays as "wall" or "unknown" (no color = not a pillar)

### Step 6: Parking marker detection

Check camera blobs for magenta:

```
blobs = [ColorBlob("red", ...), ColorBlob("orange", ...)]

magenta blobs = []  →  empty  →  parking_marker = None
```

If there were a magenta blob (e.g., `ColorBlob("magenta", angle=-10.0, area=3000)`):

```
1. Take largest magenta blob → angle=-10.0
2. Convert to LIDAR angle: -10 → 350 degrees
3. Average LIDAR readings at 345..355 degrees → say 280mm
4. parking_marker = 280.0
```

### Step 7: Floor line detection

Check camera blobs for orange/blue in the bottom of the frame:

```
blobs = [ColorBlob("red", y=210, ...), ColorBlob("orange", y=410, area=2600, ...)]

Filter rules:
  - Color must be "orange" or "blue"      → orange blob qualifies
  - y must be > height * 0.6 = 480 * 0.6 = 288    → y=410 > 288, yes
  - area must be >= 200                    → 2600 >= 200, yes

Qualifying blobs = [ColorBlob("orange", y=410, area=2600)]
Largest = "orange"
```

Result:
```python
floor_line = "orange"
```

The y-filter (bottom 40% of frame only) is important. Without it, an orange object on a shelf behind
the track could trigger a false line detection. Floor lines are on the ground, so they appear in the
bottom portion of the camera image.

### Step 8: Assemble WorldState

All results packed together:

```python
return WorldState(
    timestamp    = 1709654321.45,
    encoder_pos  = 4820,
    walls        = WallInfo(left_distance=380.0, right_distance=450.0, front_distance=2100.0),
    pillars      = [Pillar(color="red", angle=25.2, distance=310.0)],
    corner_ahead = None,
    parking_marker = None,
    floor_line   = "orange",
)
```

---

### Step 9: TrackMap update (learning)

After WorldState is produced, the controller calls `track_map.update(world_state)`.

#### Line tracking (runs every frame, every lap)

```
world.floor_line = "orange"
encoder = 4820

Is this same color as last time at nearby encoder position?
  _last_line_color = None, so no → this is new

Is _line_pair_first set?
  No → this is the first color of a new pair
  _line_pair_first = "orange"
```

Next frame, the robot drives over the blue line:

```
world.floor_line = "blue"
encoder = 4835

_line_pair_first = "orange" and color = "blue" (different!)
  → Section boundary crossed!
  → section_count = 1

Is line_direction set?
  No → first pair: "orange" first → line_direction = "CW"
  _line_pair_first = None (reset for next pair)
```

After crossing 4 pairs: `line_lap_count = 4 // 4 = 1` (one lap complete).

#### Corner recording (first lap only)

```
world.corner_ahead = None → nothing to record
```

If corner_ahead were "RIGHT" and encoder were far enough from last recorded corner:

```
corners.append(Corner(encoder_pos=4820, direction="RIGHT"))

Also finalizes the current section:
  sections.append(Section(start_encoder=3600, end_encoder=4820, width=830.0))
  (830mm average corridor width measured during this section)
```

#### Pillar recording (first lap only)

```
world.pillars = [Pillar(color="red", angle=25.2, distance=310.0)]

Is this a new pillar? Check existing records:
  No existing pillar with color="red" near encoder=4820 → yes, it's new

Record it:
  pillars.append(PillarRecord(encoder_pos=4820, color="red", side="right", angle=25.2))
  (side="right" because angle 25.2 > 0)
```

#### Lap completion check

```
len(corners) = 2 (only 2 corners so far)
Need >= 4 corners for a lap → not yet
```

---

## Chapter 3: How the Decision Layer Uses It

The decision layer receives WorldState and TrackMap and outputs just two numbers: **speed** and **steering**.

### Example: This frame

```python
world_state = WorldState(
    walls    = WallInfo(left=380, right=450, front=2100),
    pillars  = [Pillar("red", 25.2, 310.0)],
    corner   = None,
    floor_line = "orange",
    ...
)
```

State machine checks transitions:

```
1. Current state: WALL_FOLLOW
2. Is there a blocking pillar?
   Pillar at 25.2deg, threshold=35deg → |25.2| < 35 → YES, it's blocking!
3. → Transition to AVOID_PILLAR

   The pillar is RED → pass on RIGHT → steer LEFT to go around it
   speed=35, steering=45 (turned left)
```

### Example: Approaching a corner

```python
world_state = WorldState(
    walls    = WallInfo(left=380, right=450, front=350),   # front is close!
    pillars  = [],
    corner   = "RIGHT",     # right side has more space
    ...
)
```

```
1. Current state: WALL_FOLLOW
2. No blocking pillar
3. Corner approaching? front=350mm < 400mm threshold → YES
4. → Transition to CORNER
   direction = "RIGHT" → steer right
   speed=35, steering=115  (turned right)
```

### Example: Lap complete

```python
track_map.corner_count = 8      # 8 corners → corner_laps = 8//4 = 2
track_map.section_count = 9     # 9 line pairs → line_laps = 9//4 = 2
```

```
After exiting a corner:
  corner_laps = 8 // 4 = 2
  line_laps   = 9 // 4 = 2
  max(2, 2) = 2  →  lap_count = 2
  2 < 3 (target) → keep racing
```

After one more lap:

```
  corner_laps = 12 // 4 = 3
  line_laps   = 12 // 4 = 3
  max(3, 3) = 3  →  lap_count = 3
  3 >= 3 → DONE! Race complete.
```

---

## Summary: Data Flow Diagram

```
    LIDAR                   Camera                  Motor
    (USB)                   (CSI)                   (UART)
      |                       |                       |
      v                       v                       v
  dict[int,float]       list[ColorBlob]              int
  {angle: dist_mm}      [{color,angle,x,y,         encoder
                           width,height,area}]      ticks
      |                       |                       |
      +-----------+-----------+-----------+-----------+
                  |                       |
                  v                       |
          +-------+--------+              |
          | Wall Detection |              |
          |                |              |
          | scan[270]±10   |              |
          |  → left=380    |              |
          | scan[90]±10    |              |
          |  → right=450   |              |
          | scan[0]±5      |              |
          |  → front=2100  |              |
          +-------+--------+              |
                  |                        |
                  v                        |
          +-------+--------+              |
          | Corner Detect  |              |
          |                |              |
          | front < 400?   |              |
          | No → None      |              |
          +-------+--------+              |
                  |                        |
                  v                        |
          +-------+---------+             |
          | Object Cluster  |             |
          |                 |             |
          | scan → image    |             |
          | dilate → blobs  |             |
          | contours        |             |
          | classify:       |             |
          |  wall/pillar    |             |
          +-------+---------+             |
                  |                        |
                  |   +--------------------+
                  |   |
                  v   v
          +-------+---+--------+
          | Pillar Matching    |
          |                    |
          | LIDAR obj 25.5deg  |  camera red 25.2deg
          | angle diff = 0.3   |  < threshold
          | → MATCH            |
          |                    |
          | Pillar(red, 25.2,  |
          |        310mm)      |
          +-------+------------+
                  |         |
                  |         v
                  |  +------+---------+
                  |  | Parking Detect |    camera magenta blobs
                  |  | (none here)   |    → LIDAR distance lookup
                  |  +------+--------+
                  |         |
                  |         v
                  |  +------+---------+
                  |  | Floor Line     |    camera orange/blue blobs
                  |  | Detect         |    in bottom 40% of frame
                  |  | → "orange"     |
                  |  +------+--------+
                  |         |
                  +----+----+
                       |
                       v
               +-------+-------+
               |  WorldState   |          +---> TrackMap
               |               |  ------> |    (accumulates
               | walls, pillars|          |     corners, sections,
               | corner, park, |          |     pillars, lines)
               | floor_line    |          |
               +-------+-------+          +---> Visualizer
                       |                        (debug JPEG streams)
                       v
               Decision Layer
               (state machine)
                       |
                       v
               speed=35, steering=45
```
