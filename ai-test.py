import cv2
import numpy as np
from math import sqrt, pow
from ultralytics import YOLO

model = YOLO("yolov8m.pt")

cap = cv2.VideoCapture(0)

width = float(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
height = float(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
vector_cen = np.array([width/2, height/2])

deltha_dis = []
near_point = 0

while cap.isOpened():
    success, frame = cap.read()

    if success:
        # Run YOLOv8 inference on the frame
        results = model(frame, conf=0.8, imgsz = 640)

        # Visualize the results on the frame
        annotated_frame = results[0].plot()
        center_coordinates = (int(vector_cen[0]), int(vector_cen[1]))
        
        for x, r in enumerate(results):
            for y, c in enumerate(r.boxes.cls):
                boxeswh = r.boxes.xywh.detach().cpu().numpy()
                # center_obj = (int(boxeswh[y][0]), int(boxeswh[y][1]))
                # deltha_dis.append(sqrt(pow(vector_cen[0] - boxeswh[y][0], 2) + pow(vector_cen[1] - boxeswh[y][1], 2)))
                center_obj = (int(boxeswh[y][0]), int(height/2))
                deltha_dis.append(sqrt(pow(vector_cen[0] - boxeswh[y][0], 2)))
                if y == len(r.boxes.cls) - 1:
                    near_point = min(deltha_dis)
                    deltha_dis.clear()
                cv2.circle(annotated_frame, center_obj, 5, (0, 0, 0), -1)
                cv2.line(annotated_frame, center_coordinates, center_obj, (255, 0, 0), 2)
        print(near_point)
        cv2.circle(annotated_frame, center_coordinates, 5, (0, 0, 255), -1)
        # Display the annotated frame
        cv2.imshow("YOLOv8 Inference", annotated_frame)

        # Break the loop if 'q' is pressed
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break
    else:
        # Break the loop if the end of the video is reached
        break

cap.release()
cv2.destroyAllWindows()