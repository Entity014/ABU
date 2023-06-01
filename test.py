import cv2

# Initialize the webcam
cap = cv2.VideoCapture(0)

while True:
    # Read the frame from the webcam
    ret, frame = cap.read()

    # Rotate the frame by 90 degrees clockwise
    rotated_frame = cv2.rotate(frame, cv2.ROTATE_90_CLOCKWISE)

    # Display the rotated frame
    cv2.imshow("Rotated Frame", rotated_frame)

    # Break the loop if the 'q' key is pressed
    if cv2.waitKey(1) == ord("q"):
        break

# Release the webcam and close the window
cap.release()
cv2.destroyAllWindows()
