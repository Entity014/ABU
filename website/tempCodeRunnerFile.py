import cv2
from flask import Flask, render_template, Response

app = Flask(__name__)


def video_stream():
    cap = cv2.VideoCapture(0)

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        # Apply Canny edge detection to the frame
        edges = cv2.Canny(frame, 100, 200)

        # Encode the edges as JPEG
        ret, buffer = cv2.imencode(".jpg", edges)
        frame = buffer.tobytes()

        yield (b"--frame\r\n" b"Content-Type: image/jpeg\r\n\r\n" + frame + b"\r\n\r\n")

    cap.release()


@app.route("/")
def index():
    return render_template("index.html")


@app.route("/video_feed")
def video_feed():
    return Response(
        video_stream(), mimetype="multipart/x-mixed-replace; boundary=frame"
    )


if __name__ == "__main__":
    app.run(debug=True, host="0.0.0.0")
