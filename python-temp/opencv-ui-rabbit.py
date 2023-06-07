import cv2
import numpy as np
from moviepy.editor import VideoFileClip
import datetime

width, height = (780, 480)

color_zone1 = (0, 255, 0)
color_zone2 = (255, 165, 0)
color_zone3 = (255, 0, 0)


def get_gif_frame(clip, duration):
    t = cv2.getTickCount() / cv2.getTickFrequency()
    gif_frame = clip.get_frame(t % duration)
    gif_frame_resized = cv2.resize(gif_frame, (100, 100))
    return gif_frame_resized


def draw_mode(frame, block, mode):
    text_block = ["Near", "Mediam", "Enemy"]
    current_time = datetime.datetime.now().strftime("%H:%M:%S")
    text_size_time, _ = cv2.getTextSize(current_time, cv2.FONT_HERSHEY_SIMPLEX, 2.5, 3)
    for index, arr in enumerate(block):
        text = f"{text_block[index]}"
        x, y, width_x, height_y = arr
        if index == mode:
            cv2.rectangle(frame, (x, y), (x + width_x, y + height_y), (0, 255, 0), -1)
        else:
            cv2.rectangle(frame, (x, y), (x + width_x, y + height_y), (255, 0, 0), -1)
        cv2.rectangle(frame, (x, y), (x + width_x, y + height_y), (0, 0, 0), 2)

        text_size, _ = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 1.5, 3)
        text_x = x + int((width_x - text_size[0]) / 2)
        text_y = y + int((height_y + text_size[1]) / 2)
        cv2.putText(
            frame,
            text,
            (text_x, text_y),
            cv2.FONT_HERSHEY_SIMPLEX,
            2.5,
            (0, 0, 255),
            3,
        )
    cv2.putText(
        frame,
        current_time,
        (int((width - text_size_time[0]) / 2), 50),
        cv2.FONT_HERSHEY_SIMPLEX,
        1.5,
        (0, 255, 0),
        3,
        cv2.LINE_AA,
    )


def interpolate_color(start_color, end_color, value):
    b = int(start_color[0] + (end_color[0] - start_color[0]) * value)
    g = int(start_color[1] + (end_color[1] - start_color[1]) * value)
    r = int(start_color[2] + (end_color[2] - start_color[2]) * value)
    return (b, g, r)


def get_zone_color(value_pwm):
    if value_pwm <= 85:
        return color_zone1
    elif value_pwm <= 60:
        value = (value_pwm - 85) / (60 - 85)
        return interpolate_color(color_zone1, color_zone2, value)
    else:
        value = (value_pwm - 60) / (255 - 60)
        return interpolate_color(color_zone2, color_zone3, value)


def draw_power_bar(frame, value_pwm):
    pwm_height = int((value_pwm / 255) * height)
    pwm_color = get_zone_color(value_pwm)
    text_size_pwm, _ = cv2.getTextSize(f"{value_pwm}", cv2.FONT_HERSHEY_SIMPLEX, 3, 3)
    text_x_pwm = 50 + int((width - text_size_pwm[0]) / 2)
    text_y_pwm = int(height - 100) + int((100 + text_size_pwm[1]) / 2)
    cv2.rectangle(frame, (0, (height - pwm_height)), (50, height), pwm_color, -1)
    cv2.rectangle(frame, (0, 0), (50, (height - pwm_height)), (0, 0, 0), -1)
    cv2.rectangle(frame, (0, 0), (50, height), (0, 0, 0), 2)

    cv2.rectangle(
        frame,
        (50, int(height - 100)),
        (width, int(height)),
        (255, 255, 255),
        -1,
    )
    cv2.rectangle(
        frame,
        (50, int(height - 100)),
        (width, int(height)),
        (0, 0, 0),
        2,
    )
    cv2.putText(
        frame,
        f"{value_pwm}",
        (text_x_pwm, text_y_pwm),
        cv2.FONT_HERSHEY_SIMPLEX,
        3,
        (0, 0, 0),
        3,
        cv2.LINE_AA,
    )

    return frame


def main():
    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)

    percentage = 0
    block = [
        [50, 80, width - 50, 100],
        [50, 180, width - 50, 100],
        [50, 280, width - 50, 100],
    ]

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
        frame = draw_power_bar(frame, percentage)

        gif_frame = get_gif_frame(clip, duration)

        frame[
            10 : 10 + gif_frame.shape[0],
            width - 110 : width - 110 + gif_frame.shape[1],
        ] = gif_frame

        draw_mode(frame=frame, block=block, mode=0)

        cv2.rectangle(
            frame,
            (width - 110, 10),
            (width - 110 + gif_frame.shape[1], 110),
            (0, 0, 0),
            2,
        )

        frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        frame = cv2.rotate(frame, cv2.ROTATE_90_COUNTERCLOCKWISE)
        cv2.imshow("Webcam", frame)
        if cv2.waitKey(1) == ord("q"):
            break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
