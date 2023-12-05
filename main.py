import cv2
import numpy as np
import serial

# Initialize the video capture device
cap = cv2.VideoCapture(0)

# Initialize the serial port for communication with the Arduino board
ser = serial.Serial('COM4', 9600)

# Define the color ranges for the desired colors (e.g., red and green)
red_lower = np.array([0, 100, 100])
red_upper = np.array([10, 255, 255])

green_lower = np.array([35, 50, 50])
green_upper = np.array([85, 255, 255])

# Main loop
while 1:
    # Read a frame from the video capture device
    ret, frame = cap.read()

    # Convert the frame to the HSV color space
    into_hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Create masks for the desired colors
    red_mask = cv2.inRange(into_hsv, red_lower, red_upper)
    green_mask = cv2.inRange(into_hsv, green_lower, green_upper)

    # Count the number of pixels for each color
    red_count = cv2.countNonZero(red_mask)
    green_count = cv2.countNonZero(green_mask)

    # Send signals to the Arduino board based on the detected colors
    if red_count > 1000:
        ser.write(b'R')
        print("Red color detected!")
    elif green_count > 1000:
        ser.write(b'G')
        print("Green color detected!")
    else:
        ser.write(b'E')

    # Display the frame
    cv2.imshow('Original', frame)

    # Exit the loop if the user presses 'Esc'
    if cv2.waitKey(1) == 27:
        break

# Release the video capture device and close the window
cap.release()
cv2.destroyAllWindows()
