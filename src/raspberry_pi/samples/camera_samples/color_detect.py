import cv2
import numpy as np

# Try CSI camera (picamera2), fall back to USB/webcam
try:
    from picamera2 import Picamera2
    picam2 = Picamera2()
    picam2.configure(picam2.create_preview_configuration(
        main={"format": "RGB888", "size": (640, 480)}
    ))
    picam2.start()

    def read_frame():
        frame = picam2.capture_array()
        return True, cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

    print("Using CSI camera (picamera2)")
except ImportError:
    cap = cv2.VideoCapture(0)

    def read_frame():
        return cap.read()

    print("Using USB/webcam (cv2.VideoCapture)")

while True:
    ret, frame = read_frame()
    if not ret:
        break

    # Convert to HSV (easier for color detection)
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    #   Red     Orange    Yellow    Green     Cyan      Blue      Magenta   Red
    #    0        15        30        60        90       120        150      180
    #    |--------|---------|---------|---------|---------|---------|--------|

    # Red: wraps around in HSV (0-10 and 160-180)
    red_lower1 = np.array([0, 100, 100])
    red_upper1 = np.array([10, 255, 255])
    red_lower2 = np.array([160, 100, 100])
    red_upper2 = np.array([180, 255, 255])

    # Green range
    green_lower = np.array([40, 50, 50])
    green_upper = np.array([80, 255, 255])

    # Create masks
    red_mask = cv2.inRange(hsv, red_lower1, red_upper1) | cv2.inRange(hsv, red_lower2, red_upper2)
    green_mask = cv2.inRange(hsv, green_lower, green_upper)

    # Find contours
    red_contours, _ = cv2.findContours(red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    green_contours, _ = cv2.findContours(green_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Draw red objects
    for cnt in red_contours:
        area = cv2.contourArea(cnt)
        if area > 500:  # Filter noise
            x, y, w, h = cv2.boundingRect(cnt)
            cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 0, 255), 2)
            cv2.putText(frame, "RED", (x, y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

    # Draw green objects
    for cnt in green_contours:
        area = cv2.contourArea(cnt)
        if area > 500:
            x, y, w, h = cv2.boundingRect(cnt)
            cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 0), 2)
            cv2.putText(frame, "GREEN", (x, y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

    cv2.imshow("Color Detection", frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

try:
    picam2.stop()
except NameError:
    cap.release()
cv2.destroyAllWindows()
