# OpenCV Color Detection Pipeline

Step-by-step explanation of how `src/raspberry_pi/samples/camera_samples/color_detect.py` works.

## 1. Open the Camera

```python
cap = cv2.VideoCapture(0)  # 0 = default camera
```

Opens the first available camera. Returns a capture object you read frames from.

## 2. Read a Frame

```python
ret, frame = cap.read()
```

Grabs one image from the camera. `frame` is a numpy array of pixels in **BGR** format (not RGB — OpenCV uses Blue-Green-Red order). Each pixel is 3 values: `[blue, green, red]`, each 0-255.

```
frame shape: (480, 640, 3)
              height, width, channels (B, G, R)
```

## 3. Convert BGR to HSV

```python
hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
```

Converts every pixel from BGR to **HSV** (Hue, Saturation, Value). Why? Because in BGR/RGB, detecting "red" is hard — a dark red and bright red have very different values. In HSV, color is separated from brightness:

```
BGR (hard to filter by color):     HSV (easy to filter by color):
  B: 0-255                           H: 0-180  ← the color (hue)
  G: 0-255                           S: 0-255  ← how vivid
  R: 0-255                           V: 0-255  ← how bright
```

In HSV, all reds have H around 0 or 180, regardless of brightness. So you can say "give me everything with H=0-10" and you get all reds — dark, bright, pale, vivid.

## 4. Define Color Ranges

```python
# Red range 1 (H: 0-25)
red_lower1 = np.array([0, 100, 100])
red_upper1 = np.array([25, 255, 255])

# Red range 2 (H: 160-180)
red_lower2 = np.array([160, 100, 100])
red_upper2 = np.array([180, 255, 255])
```

Red wraps around the hue spectrum — it's at both ends:

```
H:  0                                          180
    |  RED  |  orange  yellow  green  blue  |RED|
    0      25                              160 180
    ├──────┤                               ├───┤
    range 1                                range 2
```

`S: 100-255` filters out pale/washed-out colors. `V: 100-255` filters out very dark pixels.

```python
green_lower = np.array([40, 50, 50])
green_upper = np.array([80, 255, 255])
```

Green is in the middle of the hue spectrum, no wrapping needed.

## 5. Create Masks

```python
red_mask = cv2.inRange(hsv, red_lower1, red_upper1) | cv2.inRange(hsv, red_lower2, red_upper2)
green_mask = cv2.inRange(hsv, green_lower, green_upper)
```

`cv2.inRange` checks every pixel: is it within the range? Returns a **binary image** (mask) — white (255) where yes, black (0) where no:

```
Original frame:          Red mask:
┌──────────────┐         ┌──────────────┐
│  green  red  │         │  ░░░░  ████  │
│         red  │         │         ████  │
│    green     │   →     │    ░░░░      │
└──────────────┘         └──────────────┘
                         ██ = white (255) = red pixel
                         ░░ = black (0)   = not red
```

The `|` (OR) combines both red ranges into one mask.

## 6. Find Contours

```python
red_contours, _ = cv2.findContours(red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
```

Finds the outlines of white regions in the mask. Each contour is a list of (x,y) points forming the boundary of one blob:

```
Red mask:                Contours found:
┌──────────────┐         ┌──────────────┐
│       ██     │         │       ┌─┐    │
│       ███    │         │       │1│    │
│              │         │              │
│   ██         │         │   ┌┐         │
│   █          │         │   │2│        │
└──────────────┘         └──────────────┘
```

- `RETR_EXTERNAL` — only outer contours (ignore holes inside blobs)
- `CHAIN_APPROX_SIMPLE` — compress straight lines to just endpoints (saves memory)

## 7. Filter by Area and Draw Boxes

```python
for cnt in red_contours:
    area = cv2.contourArea(cnt)
    if area > 500:  # Filter noise
        x, y, w, h = cv2.boundingRect(cnt)
        cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 0, 255), 2)
        cv2.putText(frame, "RED/ORANGE", (x, y-10), ...)
```

- `contourArea` — pixel count of the blob. `> 500` filters out tiny noise
- `boundingRect` — smallest axis-aligned rectangle around the contour
- `rectangle` — draws the box on the original frame. `(0, 0, 255)` is BGR for red, `2` is line thickness
- `putText` — label above the box

```
Result:
┌──────────────────┐
│                   │
│    RED/ORANGE     │
│   ┌─────────┐    │
│   │  red red │    │
│   │  red red │    │
│   └─────────┘    │
│                   │
└──────────────────┘
```

## 8. Display and Loop

```python
cv2.imshow("Color Detection", frame)

if cv2.waitKey(1) & 0xFF == ord('q'):
    break
```

- `imshow` — shows the frame with drawn boxes in a window
- `waitKey(1)` — waits 1ms for keypress. If 'q' is pressed, exit the loop
- The whole thing runs in a `while True` loop at camera FPS (~30 fps), so you see live video with bounding boxes

## 9. Cleanup

```python
cap.release()
cv2.destroyAllWindows()
```

Release the camera and close the display window.

## Summary Pipeline

```
Camera frame (BGR)
       │
       ▼
  cvtColor (BGR → HSV)
       │
       ▼
  inRange (HSV → binary mask per color)
       │
       ▼
  findContours (mask → list of outlines)
       │
       ▼
  contourArea filter (remove noise)
       │
       ▼
  boundingRect + rectangle (draw boxes on original frame)
       │
       ▼
  imshow (display result)
```
