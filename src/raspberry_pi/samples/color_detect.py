import cv2
import numpy as np

cap = cv2.VideoCapture(0)

while True:
    ret, frame = cap.read()
    if not ret:
       break

    #Convert to HSV color space
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    #   Red     Orange    Yellow    Green     Cyan      Blue      Magenta   Red
    #    0        15        30        60        90       120        150      180
    #    |--------|---------|---------|---------|---------|---------|--------|   
    # 
    
    #Red range
    red_lower1 = np.array([0, 100, 100])
    red_upper1 = np.array([10, 255, 255])
    red_lower2 = np.array([160, 100, 100])
    red_upper2 = np.array([180, 255, 255])
    
    #Green range
    green_lower1 = np.array([40, 50, 50])
    green_upper1 = np.array([80, 255, 255])

    #Create masks
    red_mask = cv2.inRange(hsv, red_lower1, red_upper1) | cv2.inRange(hsv, red_lower2, red_upper2)
    green_mask = cv2.inRange(hsv, green_lower1, green_upper1)

    #Find coutours
    red_contours, _ = cv2.findContours(red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    green_contours, _ = cv2.findContours(green_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    #Draw red objects:
    for contour in red_contours:
        area = cv2.contourArea(contour)
        if area > 500: #Filter noise
            x, y, w, h = cv2.boundingRect(contour)
            cv2.rectangle(frame, (x,y), (x+w, y+h), (0, 0, 255), 3)
            cv2.putText(frame, "RED", (x,y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
            
            
    #Draw green objects:
    for contour in green_contours:
        area = cv2.contourArea(contour)
        if area > 500: #Filter noise
            x, y, w, h = cv2.boundingRect(contour)
            cv2.rectangle(frame, (x,y), (x+w, y+h), (0, 255, 0), 3)
            cv2.putText(frame, "GREEN", (x,y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)



    cv2.imshow("Color detection", frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()