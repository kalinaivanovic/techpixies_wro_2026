"""
Pillar detection using color + shape (rectangle detection)
No ML required - pure geometry.

Logic:
1. Find colored regions (red or green)
2. Approximate contour to polygon
3. If polygon has 4 corners -> it's a rectangle -> likely a pillar
"""

import cv2
import numpy as np

cap = cv2.VideoCapture(0)


def is_rectangle(contour, min_area=500):
    """Check if contour is a rectangle (4 corners)"""
    area = cv2.contourArea(contour)
    if area < min_area:
        return False, None

    # Approximate contour to polygon
    perimeter = cv2.arcLength(contour, True)
    epsilon = 0.04 * perimeter  # Tolerance (4% of perimeter)
    approx = cv2.approxPolyDP(contour, epsilon, True)

    # Rectangle has exactly 4 corners
    if len(approx) != 4:
        return False, None

    # Check if angles are roughly 90 degrees (optional, stricter)
    # For now, just 4 corners is enough

    return True, approx


def detect_colored_rectangles(frame, hsv, color_name, lower1, upper1, lower2=None, upper2=None):
    """Find rectangles of a specific color"""
    # Create color mask
    mask = cv2.inRange(hsv, lower1, upper1)
    if lower2 is not None:
        mask |= cv2.inRange(hsv, lower2, upper2)

    # Clean up mask
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    # Find contours
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    rectangles = []
    non_rectangles = []

    for cnt in contours:
        is_rect, approx = is_rectangle(cnt)
        if is_rect:
            rectangles.append((cnt, approx))
        elif cv2.contourArea(cnt) > 500:
            non_rectangles.append(cnt)

    return rectangles, non_rectangles, mask


while True:
    ret, frame = cap.read()
    if not ret:
        break

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Red (wraps around in HSV)
    red_lower1 = np.array([0, 100, 100])
    red_upper1 = np.array([10, 255, 255])
    red_lower2 = np.array([160, 100, 100])
    red_upper2 = np.array([180, 255, 255])

    # Green
    green_lower = np.array([40, 50, 50])
    green_upper = np.array([80, 255, 255])

    # Detect
    red_rects, red_others, red_mask = detect_colored_rectangles(
        frame, hsv, "RED", red_lower1, red_upper1, red_lower2, red_upper2
    )
    green_rects, green_others, green_mask = detect_colored_rectangles(
        frame, hsv, "GREEN", green_lower, green_upper
    )

    # Draw red rectangles (PILLARS)
    for cnt, approx in red_rects:
        cv2.drawContours(frame, [approx], 0, (0, 0, 255), 3)
        x, y, w, h = cv2.boundingRect(approx)
        cv2.putText(frame, "RED PILLAR", (x, y - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

    # Draw red non-rectangles (noise - like t-shirts)
    for cnt in red_others:
        x, y, w, h = cv2.boundingRect(cnt)
        cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 0, 150), 1)
        cv2.putText(frame, "red (not rect)", (x, y - 5),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 150), 1)

    # Draw green rectangles (PILLARS)
    for cnt, approx in green_rects:
        cv2.drawContours(frame, [approx], 0, (0, 255, 0), 3)
        x, y, w, h = cv2.boundingRect(approx)
        cv2.putText(frame, "GREEN PILLAR", (x, y - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

    # Draw green non-rectangles (noise)
    for cnt in green_others:
        x, y, w, h = cv2.boundingRect(cnt)
        cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 150, 0), 1)
        cv2.putText(frame, "green (not rect)", (x, y - 5),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 150, 0), 1)

    # Stats
    cv2.putText(frame, f"Red pillars: {len(red_rects)}  |  noise: {len(red_others)}",
                (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
    cv2.putText(frame, f"Green pillars: {len(green_rects)}  |  noise: {len(green_others)}",
                (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

    cv2.imshow("Pillar Detection (Color + Shape)", frame)
    cv2.imshow("Red Mask", red_mask)
    cv2.imshow("Green Mask", green_mask)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
