import cv2
import base64
from flask import Flask, render_template, Response

app = Flask(__name__)


def capture_frames():
    cap = cv2.VideoCapture(0)

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        # Convert frame to base64
        _, buffer = cv2.imencode(".jpg", frame)
        image_base64 = base64.b64encode(buffer).decode("utf-8")

        # Yield the frame for the response
        yield (
            b"--frame\r\n"
            b"Content-Type: image/jpeg\r\n\r\n"
            + image_base64.encode("utf-8")
            + b"\r\n\r\n"
        )

    cap.release()


@app.route("/")
def index():
    return Response(
        capture_frames(), mimetype="multipart/x-mixed-replace; boundary=frame"
    )


if __name__ == "__main__":
    app.run()
