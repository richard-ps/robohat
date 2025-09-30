import cv2
import time
import numpy as np
from picamera2 import Picamera2

# Initialize the camera
picam2 = Picamera2()
picam2.preview_configuration.main.format = "RGB888"
picam2.configure("preview")
picam2.start()

im = picam2.capture_array()

hsv_frame = cv2.cvtColor(im, cv2.COLOR_BGR2HSV)

# Define specific color range to be detected
lower_bound = np.array([49, 105, 0], np.uint8)
upper_bound = np.array([94, 255, 255], np.uint8)

# Create a mask for the defined color range 
mask = cv2.inRange(hsv_frame, lower_bound, upper_bound)

# Find contours
contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

# Example: Draw a rectangle around the largest contour
if contours:
    largest_contour = max(contours, key=cv2.contourArea)
    x, y, w, h = cv2.boundingRect(largest_contour)
    cv2.rectangle(im, (x, y), (x+w, y+h), (0, 0, 255), 2)

# Display the result
cv2.imshow("Object Detection", im)
time.sleep(3)
cv2.imshow("Mask", mask)
time.sleep(10)

cv2.imwrite("/home/richard/Documents/image1.png", mask)
cv2.imwrite("/home/richard/Documents/image2.png", im)

# Exit on 'q' key press
if cv2.waitKey(10) & 0xFF == ord('q'):
    picam2.stop()
    cv2.destroyAllWindows()
