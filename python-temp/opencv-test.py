import cv2
import numpy as np

# GSTREAMER_PIPELINE = "nvarguscamerasrc ! video/x-raw(memory:NVMM), width=3280, height=2464, format=(string)NV12, framerate=21/1 ! nvvidconv flip-method=0 ! video/x-raw, width=1024, height=600, format=(string)BGRx ! videoconvert ! video/x-raw, format=(string)BGR ! appsink"

# # Create a VideoCapture object
# cap = cv2.VideoCapture(GSTREAMER_PIPELINE, cv2.CAP_GSTREAMER)
cap = cv2.VideoCapture(0)
# # Check if camera opened successfully
if cap.isOpened() == False:
    print("Unable to read camera feed")

    # Default resolutions of the frame are obtained.The default resolutions are system dependent.
    # We convert the resolutions from float to integer.
    frame_width = int(cap.get(3))
    frame_height = int(cap.get(4))

while True:
    try:
        ret, frame = cap.read()
        if ret == True:
            cv2.imshow("frame", frame)

        if cv2.waitKey(1) & 0xFF == ord("q"):
            break
    except KeyboardInterrupt:
        break

cap.release()

cv2.destroyAllWindows()
# cap = nano.Camera(width=1024, height=600, fps=30)

# while cap.isReady():
#     try:
#         frame = cap.read()
#         frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
#         cv2.imshow("frame", frame)

#         if cv2.waitKey(1) & 0xFF == ord("q"):
#             break
#     except KeyboardInterrupt:
#         break

# cap.release()
# cv2.destroyAllWindows()
