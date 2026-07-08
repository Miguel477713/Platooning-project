import os
import time
import math
import threading

# Set before importing/using VideoCapture where possible.
# These options reduce RTSP buffering/latency with OpenCV FFmpeg backend.
os.environ.setdefault(
    "OPENCV_FFMPEG_CAPTURE_OPTIONS",
    "rtsp_transport;tcp|fflags;nobuffer|flags;low_delay|max_delay;500000"
)

import cv2
import numpy as np
from flask import Flask, Response

RTSP_URL = os.getenv(
    "RTSP_URL",
    "rtsp://oliversina134:12345678@10.0.7.10:554/stream1"
)

MIN_AREA = int(os.getenv("MIN_AREA", "300"))
TARGET_FPS = float(os.getenv("TARGET_FPS", "30"))
PROCESS_WIDTH = int(os.getenv("PROCESS_WIDTH", "0"))  # 0 disables resize
JPEG_QUALITY = int(os.getenv("JPEG_QUALITY", "95"))

app = Flask(__name__)

COLORS = {
    "red": [
        ((0, 80, 80), (10, 255, 255)),
        ((170, 80, 80), (180, 255, 255)),
    ],
    "blue": [
        ((90, 80, 80), (130, 255, 255)),
    ],
    "green": [
        ((35, 60, 60), (85, 255, 255)),
    ],
}

DRAW_COLOR = (255, 255, 255)


class LatestFrameGrabber:
    """Continuously grabs frames and keeps only the newest one.

    This prevents the app from falling behind a buffered RTSP stream.
    """

    def __init__(self, url):
        self.url = url
        self.frame = None
        self.lock = threading.Lock()
        self.running = False
        self.thread = None
        self.last_capture_time = 0.0

    def Start(self):
        if self.thread and self.thread.is_alive():
            return
        self.running = True
        self.thread = threading.Thread(target=self.Loop, daemon=True)
        self.thread.start()

    def OpenCapture(self):
        cap = cv2.VideoCapture(self.url, cv2.CAP_FFMPEG)
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        return cap

    def Loop(self):
        cap = None
        while self.running:
            if cap is None or not cap.isOpened():
                cap = self.OpenCapture()
                if not cap.isOpened():
                    print("Could not open RTSP stream. Retrying...")
                    time.sleep(1.0)
                    continue

            ret, frame = cap.read()
            if not ret:
                print("RTSP read failed. Reconnecting...")
                cap.release()
                cap = None
                time.sleep(0.2)
                continue

            with self.lock:
                self.frame = frame
                self.last_capture_time = time.time()

    def GetLatest(self):
        with self.lock:
            if self.frame is None:
                return None
            return self.frame.copy()


class ProcessedJpegWorker:
    """Processes the newest captured frame at TARGET_FPS and stores JPEG bytes."""

    def __init__(self, grabber):
        self.grabber = grabber
        self.jpeg = None
        self.lock = threading.Lock()
        self.running = False
        self.thread = None
        self.processing_fps = 0.0

    def Start(self):
        self.grabber.Start()
        if self.thread and self.thread.is_alive():
            return
        self.running = True
        self.thread = threading.Thread(target=self.Loop, daemon=True)
        self.thread.start()

    def Loop(self):
        frame_period = 1.0 / max(TARGET_FPS, 1.0)
        last_fps_time = time.time()
        frames_done = 0

        while self.running:
            loop_start = time.time()
            frame = self.grabber.GetLatest()

            if frame is None:
                time.sleep(0.02)
                continue

            frame = ResizeForProcessing(frame)
            frame = ProcessFrame(frame)

            frames_done += 1
            now = time.time()
            if now - last_fps_time >= 1.0:
                self.processing_fps = frames_done / (now - last_fps_time)
                frames_done = 0
                last_fps_time = now

            cv2.putText(
                frame,
                f"processing fps: {self.processing_fps:.1f}",
                (20, frame.shape[0] - 20),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                DRAW_COLOR,
                2,
            )

            ok, buffer = cv2.imencode(
                ".jpg",
                frame,
                [int(cv2.IMWRITE_JPEG_QUALITY), JPEG_QUALITY],
            )

            if ok:
                with self.lock:
                    self.jpeg = buffer.tobytes()

            elapsed = time.time() - loop_start
            if elapsed < frame_period:
                time.sleep(frame_period - elapsed)

    def GetJPEG(self):
        with self.lock:
            return self.jpeg


def ResizeForProcessing(frame):
    if PROCESS_WIDTH <= 0:
        return frame

    height, width = frame.shape[:2]
    if width <= PROCESS_WIDTH:
        return frame

    scale = PROCESS_WIDTH / float(width)
    new_height = int(height * scale)
    return cv2.resize(frame, (PROCESS_WIDTH, new_height), interpolation=cv2.INTER_AREA)


def FindBlobCenter(hsv, ranges):
    mask_total = None

    for lower, upper in ranges:
        lower = np.array(lower, dtype=np.uint8)
        upper = np.array(upper, dtype=np.uint8)
        mask = cv2.inRange(hsv, lower, upper)
        mask_total = mask if mask_total is None else cv2.bitwise_or(mask_total, mask)

    kernel = np.ones((5, 5), np.uint8)
    mask_total = cv2.erode(mask_total, kernel, iterations=1)
    mask_total = cv2.dilate(mask_total, kernel, iterations=2)

    contours, _ = cv2.findContours(
        mask_total,
        cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_SIMPLE,
    )

    if not contours:
        return None

    largest = max(contours, key=cv2.contourArea)
    area = cv2.contourArea(largest)

    if area < MIN_AREA:
        return None

    moments = cv2.moments(largest)
    if moments["m00"] == 0:
        return None

    cx = int(moments["m10"] / moments["m00"])
    cy = int(moments["m01"] / moments["m00"])
    return cx, cy, area


def DistancePixels(p1, p2):
    dx = p2[0] - p1[0]
    dy = p2[1] - p1[1]
    d = math.sqrt(dx * dx + dy * dy)
    return d, dx, dy


def ProcessFrame(frame):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    positions = {}

    for color_name, ranges in COLORS.items():
        result = FindBlobCenter(hsv, ranges)

        if result is not None:
            cx, cy, area = result
            positions[color_name] = (cx, cy)

            cv2.circle(frame, (cx, cy), 12, DRAW_COLOR, 2)
            cv2.putText(
                frame,
                f"{color_name}: ({cx},{cy}) area={int(area)}",
                (cx + 15, cy),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                DRAW_COLOR,
                2,
            )

    names = sorted(positions.keys())
    y_text = 30

    for i in range(len(names)):
        for j in range(i + 1, len(names)):
            name_a = names[i]
            name_b = names[j]
            p1 = positions[name_a]
            p2 = positions[name_b]
            d, dx, dy = DistancePixels(p1, p2)

            cv2.line(frame, p1, p2, DRAW_COLOR, 2)

            mid_x = int((p1[0] + p2[0]) / 2)
            mid_y = int((p1[1] + p2[1]) / 2)

            cv2.putText(
                frame,
                f"{name_a}-{name_b}: {d:.1f}px",
                (mid_x, mid_y),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                DRAW_COLOR,
                2,
            )

            cv2.putText(
                frame,
                f"{name_a}-{name_b}: dx={dx}px dy={dy}px d={d:.1f}px",
                (20, y_text),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                DRAW_COLOR,
                2,
            )
            y_text += 30

    return frame


grabber = LatestFrameGrabber(RTSP_URL)
worker = ProcessedJpegWorker(grabber)


def GenerateFrames():
    worker.Start()
    delay = 1.0 / max(TARGET_FPS, 1.0)

    while True:
        frame_bytes = worker.GetJPEG()

        if frame_bytes is None:
            time.sleep(0.05)
            continue

        yield (
            b"--frame\r\n"
            b"Content-Type: image/jpeg\r\n\r\n" +
            frame_bytes +
            b"\r\n"
        )
        time.sleep(delay)


@app.route("/")
def index():
    return """
    <html>
        <body style="background-color:#111;color:white;font-family:Arial;">
            <h2>Jetson Blob Distance Stream</h2>
            <p>Using latest-frame mode: old RTSP frames are dropped instead of queued.</p>
            <img src="/video" style="max-width:100%;">
        </body>
    </html>
    """


@app.route("/video")
def video():
    return Response(
        GenerateFrames(),
        mimetype="multipart/x-mixed-replace; boundary=frame",
    )


if __name__ == "__main__":
    worker.Start()
    app.run(host="0.0.0.0", port=5000, threaded=True)
