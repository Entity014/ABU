import cv2
import numpy as np
from math import sqrt, pow

cap = cv2.VideoCapture('abu.mp4')

width = float(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
height = float(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
vector_cen = np.array([width/2, height/2])

deltha_dis = []
near_point = 0

temp = 0
lower_blue = np.array([19, 89, 88])
upper_blue = np.array([28, 255, 255])

while True:
    num = 0
    # Capture frame-by-frame
    ret, frame = cap.read()

    # Convert BGR to HSV
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Define range of blue color in HSV

    # Threshold the HSV image to get only blue colors
    mask = cv2.inRange(hsv, lower_blue, upper_blue)

    # Bitwise-AND mask and original image
    res = cv2.bitwise_and(frame,frame, mask= mask)

    # Find contours
    contours, hierarchy = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    center_coordinates = (int(vector_cen[0]), int(vector_cen[1]))

    # Loop through contours and detect shapes
    for cnt in contours:
        approx = cv2.approxPolyDP(cnt,0.01*cv2.arcLength(cnt,True),True)
        area = cv2.contourArea(cnt)

        if area > 500:
            x,y,w,h = cv2.boundingRect(cnt)
            if len(approx) > 5:
                num +=1
                center_obj = (int(x + (w/2)), int(height/2))
                deltha_dis.append(sqrt(pow(vector_cen[0] - (x + (w/2)), 2)))
                if num > temp:
                    temp = num
                elif num == 1:
                    temp = num
                    near_point = min(deltha_dis)
                    deltha_dis.clear()
                
                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                cv2.drawContours(frame,[cnt],0,(255,255,0),-1)
                cv2.putText(frame,'Circle',(x,y),cv2.FONT_HERSHEY_SIMPLEX, 1,(255,0,0),2)
                cv2.circle(frame, center_obj, 5, (0, 0, 0), -1)
                cv2.line(frame, center_coordinates, center_obj, (255, 0, 0), 2)

    print(near_point)
    cv2.circle(frame, center_coordinates, 5, (0, 0, 255), -1)
    # Display the resulting frame
    cv2.imshow('frame',frame)
    cv2.imshow('mask',mask)
    cv2.imshow('res',res)

    # Quit program when 'q' key is pressed
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
    if cap.get(cv2.CAP_PROP_POS_FRAMES) == cap.get(cv2.CAP_PROP_FRAME_COUNT):
        cap.set(cv2.CAP_PROP_POS_FRAMES, 0)

# Release the capture
cap.release()
cv2.destroyAllWindows()
