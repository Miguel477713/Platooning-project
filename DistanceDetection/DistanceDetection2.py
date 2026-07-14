import os
import time
import math
import threading
import argparse
import json
from pathlib import Path

# Set before importing/using VideoCapture where possible.
# These options reduce RTSP buffering/latency with OpenCV FFmpeg backend.
os.environ.setdefault(
    "OPENCV_FFMPEG_CAPTURE_OPTIONS",
    "rtsp_transport;tcp|fflags;nobuffer|flags;low_delay|max_delay;500000"
)

import cv2
import numpy as np
from flask import Flask, Response, jsonify, request

RTSP_URL = os.getenv(
    "RTSP_URL",
    "rtsp://oliversina135:12345678@10.0.7.12:554/stream1"
)

MIN_AREA = int(os.getenv("MIN_AREA", "300"))
TARGET_FPS = float(os.getenv("TARGET_FPS", "30"))
PROCESS_WIDTH = int(os.getenv("PROCESS_WIDTH", "0"))  # 0 disables resize
JPEG_QUALITY = int(os.getenv("JPEG_QUALITY", "95"))
DEFAULT_CALIBRATION_DISTANCE = 100.0
CALIBRATION_FILE = Path(
    os.getenv(
        "CALIBRATION_FILE",
        Path.home() / ".local" / "share" / "distance_detection2" / "calibration.json",
    )
)

app = Flask(__name__)
calibration = None
clicked_points = []
calibration_lock = threading.Lock()
calibration_mode = False

COLORS = {
    # HSV ranges tuned from rosa.png (#e8428c) and verde.png (#679341).
    "pink": [
        ((160, 100, 100), (175, 255, 255)),
    ],
    "green": [
        ((38, 70, 70), (55, 255, 255)),
        ((38, 49, 87), (54, 189, 227))
    ],
}

DRAW_COLOR = (255, 255, 255)
CALIBRATION_COLOR = (255, 0, 0)


class HomographyCalibration:
    """Maps four user-selected image corners onto a measured rectangle."""

    def __init__(self, top, right, bottom, left, unit):
        self.top = top
        self.right = right
        self.bottom = bottom
        self.left = left
        self.unit = unit
        self.transform = None
        self.ordered_corners = None
        self.calibrated_at = None
        # A homography needs rectangular world coordinates. Averaging opposite
        # sides tolerates small measuring errors while preserving that shape.
        self.width = (top + bottom) / 2.0
        self.height = (right + left) / 2.0

    def GetTransform(self, points):
        if len(points) != 4:
            return None, None

        # Points arrive in the UI's required order: top-left, top-right,
        # bottom-right, bottom-left. Keeping that order avoids guessing it
        # from a perspective-distorted camera image.
        source_points = np.float32(points)
        destination_points = np.float32([
            [0, 0],
            [self.width, 0],
            [self.width, self.height],
            [0, self.height],
        ])
        transform = cv2.getPerspectiveTransform(source_points, destination_points)
        return transform, source_points

    def Distance(self, transform, point_a, point_b):
        image_points = np.float32([[point_a], [point_b]])
        world_points = cv2.perspectiveTransform(image_points, transform).reshape(2, 2)
        return float(np.linalg.norm(world_points[1] - world_points[0]))

    def TryInitialize(self, points):
        if self.transform is not None or len(points) != 4:
            return False

        transform, ordered_corners = self.GetTransform(points)
        if transform is None:
            return False

        self.transform = transform
        self.ordered_corners = ordered_corners
        self.calibrated_at = time.time()
        return True

    def GetCachedTransform(self):
        return self.transform, self.ordered_corners


def SaveCalibration(points, current_calibration):
    """Persist the click order and real-world measurements for later runs."""
    payload = {
        "points": points,
        "top": current_calibration.top,
        "right": current_calibration.right,
        "bottom": current_calibration.bottom,
        "left": current_calibration.left,
        "unit": current_calibration.unit,
    }
    CALIBRATION_FILE.parent.mkdir(parents=True, exist_ok=True)
    CALIBRATION_FILE.write_text(json.dumps(payload), encoding="utf-8")


def LoadCalibration():
    if not CALIBRATION_FILE.is_file():
        return None, []

    try:
        payload = json.loads(CALIBRATION_FILE.read_text(encoding="utf-8"))
        points = [tuple(point) for point in payload["points"]]
        current_calibration = HomographyCalibration(
            PositiveDistance(payload["top"], "Top"),
            PositiveDistance(payload["right"], "Right"),
            PositiveDistance(payload["bottom"], "Bottom"),
            PositiveDistance(payload["left"], "Left"),
            str(payload.get("unit", "cm")),
        )
        if not current_calibration.TryInitialize(points):
            raise ValueError("Stored calibration does not contain four valid points.")
        return current_calibration, points
    except (KeyError, TypeError, ValueError, json.JSONDecodeError, argparse.ArgumentTypeError) as error:
        print(f"Ignoring invalid stored calibration: {error}")
        return None, []


def OrderRectangleCorners(points):
    """Return four image points as top-left, top-right, bottom-right, bottom-left."""
    points = np.float32(points)
    sorted_by_y = points[np.argsort(points[:, 1])]
    top = sorted_by_y[:2]
    bottom = sorted_by_y[2:]

    top = top[np.argsort(top[:, 0])]
    bottom = bottom[np.argsort(bottom[:, 0])]
    return np.float32([top[0], top[1], bottom[1], bottom[0]])


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


def BuildColorMask(hsv, ranges):
    mask_total = None

    for lower, upper in ranges:
        lower = np.array(lower, dtype=np.uint8)
        upper = np.array(upper, dtype=np.uint8)
        mask = cv2.inRange(hsv, lower, upper)
        mask_total = mask if mask_total is None else cv2.bitwise_or(mask_total, mask)

    kernel = np.ones((5, 5), np.uint8)
    mask_total = cv2.erode(mask_total, kernel, iterations=1)
    mask_total = cv2.dilate(mask_total, kernel, iterations=2)
    return mask_total


def FindBlobCenters(hsv, ranges):
    mask_total = BuildColorMask(hsv, ranges)

    contours, _ = cv2.findContours(
        mask_total,
        cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_SIMPLE,
    )

    centers = []
    for contour in contours:
        area = cv2.contourArea(contour)
        if area < MIN_AREA:
            continue

        moments = cv2.moments(contour)
        if moments["m00"] == 0:
            continue

        cx = int(moments["m10"] / moments["m00"])
        cy = int(moments["m01"] / moments["m00"])
        centers.append((cx, cy, area))

    return sorted(centers, key=lambda item: item[2], reverse=True)


def FindBlobCenter(hsv, ranges):
    centers = FindBlobCenters(hsv, ranges)
    return centers[0] if centers else None


def DistancePixels(p1, p2):
    dx = p2[0] - p1[0]
    dy = p2[1] - p1[1]
    d = math.sqrt(dx * dx + dy * dy)
    return d, dx, dy


def ProcessFrame(frame):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    positions = {}

    with calibration_lock:
        active_calibration = calibration
        points_to_draw = list(clicked_points)

    transform = None
    ordered_corners = None
    if active_calibration is not None:
        transform, ordered_corners = active_calibration.GetCachedTransform()

    if transform is not None:
        status_text = "calibration: active"
    elif len(points_to_draw) == 4:
        status_text = "calibration: enter side lengths and apply"
    else:
        status_text = f"calibration: click corner {len(points_to_draw) + 1} of 4"

    click_names = ("top-left", "top-right", "bottom-right", "bottom-left")
    for index, point in enumerate(points_to_draw):
        cv2.circle(frame, point, 10, CALIBRATION_COLOR, 2)
        cv2.putText(
            frame,
            click_names[index],
            (point[0] + 12, point[1] - 10),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            CALIBRATION_COLOR,
            2,
        )

    if ordered_corners is not None:
        polygon = ordered_corners.astype(np.int32).reshape((-1, 1, 2))
        cv2.polylines(frame, [polygon], True, CALIBRATION_COLOR, 2)
        corner_names = ("top-left", "top-right", "bottom-right", "bottom-left")
        for index, point in enumerate(ordered_corners.astype(np.int32), start=1):
            point_tuple = tuple(point)
            cv2.circle(frame, point_tuple, 10, CALIBRATION_COLOR, 2)
            cv2.putText(
                frame,
                corner_names[index - 1],
                (point_tuple[0] + 12, point_tuple[1] - 10),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                CALIBRATION_COLOR,
                2,
            )

        edge_lengths = (
            active_calibration.width,
            active_calibration.height,
            active_calibration.width,
            active_calibration.height,
        )
        ordered_points = ordered_corners.astype(np.int32)
        for index, edge_length in enumerate(edge_lengths):
            point_a = ordered_points[index]
            point_b = ordered_points[(index + 1) % 4]
            midpoint = ((point_a + point_b) / 2).astype(int)
            cv2.putText(
                frame,
                f"{edge_length:g} {active_calibration.unit}",
                tuple(midpoint),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                CALIBRATION_COLOR,
                2,
            )

    cv2.putText(
        frame,
        status_text,
        (20, 30),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.7,
        CALIBRATION_COLOR,
        2,
    )

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
    y_text = 60

    for i in range(len(names)):
        for j in range(i + 1, len(names)):
            name_a = names[i]
            name_b = names[j]
            p1 = positions[name_a]
            p2 = positions[name_b]
            cv2.line(frame, p1, p2, DRAW_COLOR, 2)

            mid_x = int((p1[0] + p2[0]) / 2)
            mid_y = int((p1[1] + p2[1]) / 2)
            is_green_pink_pair = {name_a, name_b} == {"green", "pink"}

            if active_calibration is not None and transform is not None and is_green_pink_pair:
                real_distance = active_calibration.Distance(transform, p1, p2)
                distance_label = (
                    f"{name_a}-{name_b}: {real_distance:.2f} {active_calibration.unit}"
                )
            elif is_green_pink_pair:
                distance_label = f"{name_a}-{name_b}: calibration needed"
            else:
                continue

            cv2.putText(
                frame,
                distance_label,
                (mid_x, mid_y),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                DRAW_COLOR,
                2,
            )

            cv2.putText(
                frame,
                distance_label,
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


def PositiveDistance(value, label):
    try:
        distance = float(value)
    except (TypeError, ValueError) as error:
        raise argparse.ArgumentTypeError(
            f"{label} must be a positive number."
        ) from error

    if distance <= 0:
        raise argparse.ArgumentTypeError(f"{label} must be greater than zero.")
    return distance


def CalibrationState():
    with calibration_lock:
        return {
            "points": list(clicked_points),
            "calibrated": calibration is not None and calibration.transform is not None,
            "calibration_mode": calibration_mode,
        }


@app.get("/calibration/state")
def GetCalibrationState():
    return jsonify(CalibrationState())


@app.post("/calibration/click")
def AddCalibrationClick():
    data = request.get_json(silent=True) or {}
    try:
        point = (int(round(float(data["x"]))), int(round(float(data["y"]))))
    except (KeyError, TypeError, ValueError):
        return jsonify(error="Coordinates must be numbers."), 400

    with calibration_lock:
        if not calibration_mode:
            return jsonify(error="Restart with --calibrate to change the saved calibration."), 403
        if len(clicked_points) >= 4:
            return jsonify(error="Four points are already selected. Reset to recalibrate."), 409
        clicked_points.append(point)

    return jsonify(CalibrationState())


@app.post("/calibration/apply")
def ApplyCalibration():
    global calibration
    data = request.get_json(silent=True) or {}
    try:
        top = PositiveDistance(data["top"], "Top")
        right = PositiveDistance(data["right"], "Right")
        bottom = PositiveDistance(data["bottom"], "Bottom")
        left = PositiveDistance(data["left"], "Left")
    except (KeyError, argparse.ArgumentTypeError) as error:
        return jsonify(error=str(error)), 400

    unit = str(data.get("unit", "cm")).strip() or "cm"
    with calibration_lock:
        if not calibration_mode:
            return jsonify(error="Restart with --calibrate to change the saved calibration."), 403
        if len(clicked_points) != 4:
            return jsonify(error="Click all four corners before applying calibration."), 400
        new_calibration = HomographyCalibration(top, right, bottom, left, unit)
        if not new_calibration.TryInitialize(clicked_points):
            return jsonify(error="Could not create the calibration transform."), 400
        calibration = new_calibration
        try:
            SaveCalibration(clicked_points, calibration)
        except OSError as error:
            return jsonify(error=f"Calibration is active but could not be saved: {error}"), 500

    return jsonify(CalibrationState())


@app.post("/calibration/reset")
def ResetCalibration():
    global calibration
    with calibration_lock:
        if not calibration_mode:
            return jsonify(error="Restart with --calibrate to change the saved calibration."), 403
        clicked_points.clear()
        calibration = None
    return jsonify(CalibrationState())


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
    <!doctype html>
    <html lang="en">
    <head>
        <meta charset="utf-8">
        <meta name="viewport" content="width=device-width, initial-scale=1">
        <title>Distance Calibration</title>
        <style>
            body { margin: 0; background: #181818; color: #eee; font: 16px Arial, sans-serif; }
            main { width: 100vw; }
            #toolbar { padding: 10px 16px; }
            #toolbar h2, #toolbar p { margin: 0 0 8px; }
            img { display: block; width: 100%; max-height: calc(100vh - 120px); cursor: crosshair; object-fit: contain; }
            fieldset { margin-top: 12px; max-width: 680px; }
            input { width: 80px; margin: 4px; }
            button { margin: 8px 8px 0 0; padding: 8px 12px; }
        </style>
    </head>
    <body>
        <main>
            <div id="toolbar">
            <h2>Pink-Green Distance</h2>
            <p id="status">Loading calibration...</p>
            <button id="fullscreen" type="button">Fullscreen video</button>
            <div id="controls">
            <img id="stream" src="/video" alt="Live video stream">
            <fieldset id="measurement-controls">
                <legend>Measured rectangle sides</legend>
                <label>Top <input id="top" type="number" min="0.001" step="any" value="100"></label>
                <label>Right <input id="right" type="number" min="0.001" step="any" value="100"></label>
                <label>Bottom <input id="bottom" type="number" min="0.001" step="any" value="100"></label>
                <label>Left <input id="left" type="number" min="0.001" step="any" value="100"></label>
                <label>Unit <input id="unit" value="cm"></label><br>
                <button id="apply">Apply calibration</button>
                <button id="reset">Reset points</button>
            </fieldset>
            </div>
            </div>
        </main>
        <script>
            const stream = document.getElementById('stream');
            const status = document.getElementById('status');
            const controls = document.getElementById('measurement-controls');
            const cornerNames = ['top-left', 'top-right', 'bottom-right', 'bottom-left'];

            function showState(state) {
                controls.hidden = !state.calibration_mode;
                if (!state.calibration_mode && state.calibrated) {
                    status.textContent = 'Using the saved calibration. Restart with --calibrate to change it.';
                } else if (state.calibrated) {
                    status.textContent = 'Calibration active.';
                } else if (state.points.length < 4) {
                    status.textContent = `Click ${cornerNames[state.points.length]} (${state.points.length + 1} of 4).`;
                } else {
                    status.textContent = 'Enter the measured side lengths, then apply calibration.';
                }
            }

            async function post(url, body) {
                const response = await fetch(url, {
                    method: 'POST',
                    headers: { 'Content-Type': 'application/json' },
                    body: JSON.stringify(body || {}),
                });
                const data = await response.json();
                if (!response.ok) throw new Error(data.error || 'Request failed.');
                return data;
            }

            async function refreshState() {
                const response = await fetch('/calibration/state');
                showState(await response.json());
            }

            stream.addEventListener('click', async (event) => {
                const rect = stream.getBoundingClientRect();
                const x = (event.clientX - rect.left) * stream.naturalWidth / rect.width;
                const y = (event.clientY - rect.top) * stream.naturalHeight / rect.height;
                try { showState(await post('/calibration/click', { x, y })); }
                catch (error) { status.textContent = error.message; }
            });

            document.getElementById('apply').addEventListener('click', async () => {
                const body = {};
                for (const id of ['top', 'right', 'bottom', 'left', 'unit']) {
                    body[id] = document.getElementById(id).value;
                }
                try { showState(await post('/calibration/apply', body)); }
                catch (error) { status.textContent = error.message; }
            });

            document.getElementById('reset').addEventListener('click', async () => {
                try { showState(await post('/calibration/reset')); }
                catch (error) { status.textContent = error.message; }
            });

            document.getElementById('fullscreen').addEventListener('click', () => {
                stream.requestFullscreen();
            });

            refreshState();
        </script>
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
    parser = argparse.ArgumentParser(
        description="Measure pink-green distance using a saved click calibration."
    )
    parser.add_argument(
        "--calibrate",
        action="store_true",
        help="Select four new corners and enter their measured side lengths.",
    )
    args = parser.parse_args()

    if args.calibrate:
        calibration_mode = True
        print("Calibration mode: select four corners in the browser.")
    else:
        calibration, stored_points = LoadCalibration()
        if calibration is None:
            calibration_mode = True
            print("No saved calibration found. Calibration is required in the browser.")
        else:
            clicked_points[:] = stored_points
            print(f"Loaded saved calibration from {CALIBRATION_FILE}")

    worker.Start()
    app.run(host="0.0.0.0", port=5000, threaded=True)
