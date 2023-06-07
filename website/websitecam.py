import cv2
from flask import Flask, render_template, Response
from threading import Thread
import time

app = Flask(__name__)


def video_stream():
    cap = cv2.VideoCapture(0)
    cap.set(3, 320)  # Set width
    cap.set(4, 240)  # Set height

    while True:
        start_time = time.time()

        ret, frame = cap.read()
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        if not ret:
            break

        # Apply Gaussian blur effect to the frame
        blurred_frame = cv2.GaussianBlur(frame, (5, 5), 0)

        # Apply bilateral filter to reduce noise
        denoised_frame = cv2.bilateralFilter(blurred_frame, 9, 75, 75)

        # Apply Canny edge detection to the denoised frame
        edges = cv2.Canny(denoised_frame, 50, 150)

        # Encode the mask as JPEG
        ret, buffer = cv2.imencode(".jpg", edges)
        frame = buffer.tobytes()

        yield (b"--frame\r\n" b"Content-Type: image/jpeg\r\n\r\n" + frame + b"\r\n\r\n")

        # Introduce a delay to control the frame rate
        elapsed_time = time.time() - start_time
        delay = max(0, 0.05 - elapsed_time)  # Adjust the delay time as needed
        time.sleep(delay)


@app.route("/")
def index():
    return render_template("index.html")


@app.route("/video_feed")
def video_feed():
    return Response(
        video_stream(), mimetype="multipart/x-mixed-replace; boundary=frame"
    )


if __name__ == "__main__":
    # Start video capture in a separate thread
    video_thread = Thread(target=video_stream)
    video_thread.daemon = True
    video_thread.start()

    app.run(debug=True, host="0.0.0.0")
