import cv2
import numpy as np
import math

# Initialize the camera
cap = cv2.VideoCapture("tesvideo.mp4")

while True:
    # Capture a frame from the camera
    ret, frame = cap.read()
    
    # Create a canvas to draw on
    h, w, _ = frame.shape
    canvas = np.zeros((h, w, 3), dtype=np.uint8)
    
    # Draw a sine wave
    for x in range(w):
        y = int(h/2 + h/4 * math.sin(2 * math.pi * x / w))
        cv2.circle(canvas, (x, y), 2, (0, 255, 0), -1)
    
    # Draw a cosine wave
    for x in range(w):
        y = int(h/2 + h/4 * math.cos(2 * math.pi * x / w))
        cv2.circle(canvas, (x, y), 2, (255, 0, 0), -1)
    
    # Overlay the canvas on the camera feed
    output = cv2.addWeighted(frame, 0.8, canvas, 0.2, 0)
    
    # Display the output
    cv2.imshow('Trigonometric Functions', output)
    
    # Exit the loop if 'q' is pressed
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the camera and close all windows
cap.release()
cv2.destroyAllWindows()