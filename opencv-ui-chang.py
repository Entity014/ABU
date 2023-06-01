import cv2
import numpy as np
from moviepy.editor import VideoFileClip
import datetime


def get_gif_frame(clip, duration):
    t = cv2.getTickCount() / cv2.getTickFrequency()
    gif_frame = clip.get_frame(t % duration)
    gif_frame_resized = cv2.resize(gif_frame, (100, 100))
    return gif_frame_resized


def draw_gauge(frame, x_cord, y_cord, thickness, radius, circle_colour, value_pwm=0):
    value_pwm = min(value_pwm, 255)

    fill_angle = int(value_pwm * 280 / 255)
    per = value_pwm
    if per > 255:
        per = 255
    ac = (int(per * 255 / 255), int(255 - per * 100 / 255), int(0))

    pertext = str(value_pwm)
    text_size, _ = cv2.getTextSize(pertext, cv2.FONT_HERSHEY_SIMPLEX, 1.5, 3)
    text_width = text_size[0]
    text_height = text_size[1]
    text_x = int(x_cord) - text_width // 2
    text_y = int(y_cord) + text_height // 2
    cv2.putText(
        frame,
        pertext,
        (text_x, text_y),
        cv2.FONT_HERSHEY_SIMPLEX,
        1.5,
        ac,
        3,
        cv2.LINE_AA,
    )

    for i in range(0, thickness):
        start_angle = -225
        end_angle = start_angle + 270
        cv2.ellipse(
            frame,
            (int(x_cord), int(y_cord)),
            (radius - i, radius - i),
            0,
            start_angle,
            end_angle,
            circle_colour,
            thickness=1,
        )
        if value_pwm > 0:
            cv2.ellipse(
                frame,
                (int(x_cord), int(y_cord)),
                (radius - i, radius - i),
                0,
                start_angle,
                start_angle + fill_angle - 8,
                ac,
                thickness=1,
            )


def draw_mode(frame, block, mode):
    text_block = ["N", "M", "F"]
    current_time = datetime.datetime.now().strftime("%H:%M:%S")
    text_size_time, _ = cv2.getTextSize(current_time, cv2.FONT_HERSHEY_SIMPLEX, 1.5, 3)
    for index, arr in enumerate(block):
        text = f"{text_block[index]}"
        x, y, width, height = arr
        if index == mode:
            cv2.rectangle(frame, (x, y), (x + width, y + height), (208, 245, 190), -1)
        else:
            cv2.rectangle(frame, (x, y), (x + width, y + height), (253, 206, 223), -1)
        cv2.rectangle(frame, (x, y), (x + width, y + height), (0, 0, 0), 2)

        text_size, _ = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 1.5, 3)
        text_x = x + int((width - text_size[0]) / 2)
        text_y = y + int((height + text_size[1]) / 2)
        cv2.putText(
            frame,
            text,
            (text_x, text_y),
            cv2.FONT_HERSHEY_SIMPLEX,
            1.5,
            (25, 55, 109),
            3,
        )
    cv2.putText(
        frame,
        current_time,
        (int((760 - text_size_time[0]) / 2), 50),
        cv2.FONT_HERSHEY_SIMPLEX,
        1.5,
        (0, 255, 0),
        3,
        cv2.LINE_AA,
    )


def main():
    width, height = (760, 480)

    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)

    percentage = 0
    block = [
        [0, int(height - 50), 100, 50],
        [0, int(height - 100), 100, 50],
        [0, int(height - 150), 100, 50],
    ]

    circle_c = (55, 77, 91)

    gif_path = (
        "fast-cat-cat-excited.gif"  # Replace with the actual path to your GIF file
    )
    clip = VideoFileClip(gif_path)

    duration = clip.duration
    while True:
        ret, frame = cap.read()

        if not ret:
            break

        # FOR SHOWING CHANGE IN GAUGE
        percentage += 1
        if percentage > 255:
            percentage = 0

        frame = cv2.resize(frame, (width, height))
        frame = cv2.flip(frame, 1)
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

        gif_frame = get_gif_frame(clip, duration)

        frame[
            10 : 10 + gif_frame.shape[0],
            10 : 10 + gif_frame.shape[1],
        ] = gif_frame

        draw_gauge(
            frame=frame,
            x_cord=width * 0.8,
            y_cord=height * 0.8,
            thickness=40,
            radius=100,
            circle_colour=circle_c,
            value_pwm=percentage,
        )
        draw_mode(frame=frame, block=block, mode=0)

        cv2.rectangle(frame, (10, 10), (110, 110), (0, 0, 0), 2)

        frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        cv2.namedWindow("Webcam", cv2.WND_PROP_FULLSCREEN)
        cv2.setWindowProperty("Webcam", cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)
        cv2.imshow("Webcam", frame)
        if cv2.waitKey(1) == ord("q"):
            break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
