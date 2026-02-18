"""
Interactive HSV range picker.

Point the camera at an object, adjust sliders until only
the target object is white in the mask. Read H/S/V values
from the trackbars and use them in your detection code.

Controls:
  - Adjust sliders to narrow down the color range
  - 'q' to quit
  - 'p' to print current values to terminal
"""
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

cv2.namedWindow("Trackbars")
cv2.createTrackbar("H Min", "Trackbars", 0, 180, lambda x: None)
cv2.createTrackbar("H Max", "Trackbars", 10, 180, lambda x: None)
cv2.createTrackbar("S Min", "Trackbars", 100, 255, lambda x: None)
cv2.createTrackbar("S Max", "Trackbars", 255, 255, lambda x: None)
cv2.createTrackbar("V Min", "Trackbars", 100, 255, lambda x: None)
cv2.createTrackbar("V Max", "Trackbars", 255, 255, lambda x: None)

while True:
    ret, frame = read_frame()
    if not ret:
        break

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    h_min = cv2.getTrackbarPos("H Min", "Trackbars")
    h_max = cv2.getTrackbarPos("H Max", "Trackbars")
    s_min = cv2.getTrackbarPos("S Min", "Trackbars")
    s_max = cv2.getTrackbarPos("S Max", "Trackbars")
    v_min = cv2.getTrackbarPos("V Min", "Trackbars")
    v_max = cv2.getTrackbarPos("V Max", "Trackbars")

    lower = np.array([h_min, s_min, v_min])
    upper = np.array([h_max, s_max, v_max])

    mask = cv2.inRange(hsv, lower, upper)
    result = cv2.bitwise_and(frame, frame, mask=mask)

    # Show current values on the frame
    text = f"H:[{h_min}-{h_max}] S:[{s_min}-{s_max}] V:[{v_min}-{v_max}]"
    cv2.putText(result, text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

    cv2.imshow("Original", frame)
    cv2.imshow("Mask", mask)
    cv2.imshow("Result", result)

    key = cv2.waitKey(1) & 0xFF
    if key == ord('q'):
        break
    elif key == ord('p'):
        print(f"lower = np.array([{h_min}, {s_min}, {v_min}])")
        print(f"upper = np.array([{h_max}, {s_max}, {v_max}])")
        print()

try:
    picam2.stop()
except NameError:
    cap.release()
cv2.destroyAllWindows()
