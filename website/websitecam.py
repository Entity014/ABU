import cv2
from flask import Flask, render_template, Response

app = Flask(__name__)
cap = cv2.VideoCapture(0)


# Function to pixelate the input frame
def pixelate_frame(frame, pixel_size):
    # Get frame size
    height, width = frame.shape[:2]

    # Resize frame to desired pixelated size
    temp = cv2.resize(frame, (pixel_size, pixel_size), interpolation=cv2.INTER_LINEAR)

    # Resize frame back to original size
    pixelated_frame = cv2.resize(temp, (width, height), interpolation=cv2.INTER_NEAREST)

    return pixelated_frame


def generate_frames():
    while True:
        # Read a frame from the webcam
        ret, frame = cap.read()

        # Check if frame is successfully read
        if not ret:
            break

        # Apply pixelation to the frame
        pixelated_frame = pixelate_frame(frame, pixel_size=32)

        # Encode the frame as JPEG
        ret, buffer = cv2.imencode(".jpg", pixelated_frame)
        frame = buffer.tobytes()

        # Yield the frame in a response
        yield (b"--frame\r\n" b"Content-Type: image/jpeg\r\n\r\n" + frame + b"\r\n\r\n")


@app.route("/")
def index():
    return render_template("index.html")


@app.route("/video_feed")
def video_feed():
    return Response(
        generate_frames(), mimetype="multipart/x-mixed-replace; boundary=frame"
    )


if __name__ == "__main__":
    app.run(debug=True, host="0.0.0.0")
