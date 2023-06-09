import cv2
import numpy as np

cap = cv2.VideoCapture(0)

width = float(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
height = float(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
vector_cen = np.array([width / 2, height / 2])

deltha_dis_min = []
deltha_dis = []
near_point = 0
abs_near_point = 0

pre_num = 0
lower_range = np.array([84, 90, 127])
upper_range = np.array([105, 255, 255])
# lower_range = np.array([19, 89, 88])
# upper_range = np.array([28, 255, 255])

save_cnt = []


def getCnt(frame, mode: int):
    if mode == 0:
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, lower_range, upper_range)
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    elif mode == 1:
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (7, 7), 1)
        imgCanny = cv2.Canny(blurred, 50, 150)
        kernel = np.ones((5, 5))
        mask = cv2.dilate(imgCanny, kernel, iterations=1)
    contours, hierarchy = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    return contours, hierarchy, mask


while True:
    num = 0
    # Capture frame-by-frame
    ret, frame = cap.read()

    contours, hierarchy, mask = getCnt(frame, 0)

    center_coordinates = (int(vector_cen[0]), int(vector_cen[1]))

    # Loop through contours and detect shapes
    for cnt in contours:
        approx = cv2.approxPolyDP(cnt, 0.01 * cv2.arcLength(cnt, True), True)
        area = cv2.contourArea(cnt)
        x, y, w, h = cv2.boundingRect(cnt)
        if (area > 500) and (h > w):  # and (len(approx) >= 4) and (len(approx) <= 10):
            center_obj = (int(x + (w / 2)), int(height / 2))
            # distance_point = sqrt(pow(vector_cen[0] - (x + (w/2)), 2))
            distance_point = vector_cen[0] - (x + (w / 2))
            deltha_dis_min.append(abs(distance_point))
            deltha_dis.append(distance_point)
            save_cnt.append(cnt)

            if num == 0:
                abs_near_point = min(deltha_dis_min)
                near_point = deltha_dis[deltha_dis_min.index(abs_near_point)]
                cv2.drawContours(
                    frame,
                    [save_cnt[deltha_dis_min.index(abs_near_point)]],
                    0,
                    (255, 255, 0),
                    -1,
                )
                deltha_dis.clear()
                deltha_dis_min.clear()
                save_cnt.clear()

            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 0, 0), 2)
            # cv2.drawContours(frame, [cnt], 0, (255, 255, 0), -1)
            cv2.putText(
                frame,
                f"Pole<{num}> {len(approx)} [ {(distance_point):.2f}]",
                (x, y),
                cv2.FONT_HERSHEY_SIMPLEX,
                1,
                (255, 0, 255),
                2,
            )
            cv2.circle(frame, center_obj, 5, (0, 0, 0), -1)
            cv2.line(frame, center_coordinates, center_obj, (255, 0, 0), 2)
            num += 1

    print(f"{near_point}, {abs_near_point}")
    cv2.circle(frame, center_coordinates, 5, (0, 0, 255), -1)
    # Display the resulting frame
    cv2.imshow("frame", frame)
    cv2.imshow("mask", mask)

    # Quit program when 'q' key is pressed
    if cv2.waitKey(1) & 0xFF == ord("q"):
        break
    if cap.get(cv2.CAP_PROP_POS_FRAMES) == cap.get(cv2.CAP_PROP_FRAME_COUNT):
        cap.set(cv2.CAP_PROP_POS_FRAMES, 0)

# Release the capture
cap.release()
cv2.destroyAllWindows()
