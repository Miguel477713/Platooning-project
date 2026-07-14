import os
import time
import math
import threading
import argparse

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
    "rtsp://oliversina135:12345678@10.0.7.12:554/stream1"
)

MIN_AREA = int(os.getenv("MIN_AREA", "300"))
TARGET_FPS = float(os.getenv("TARGET_FPS", "30"))
PROCESS_WIDTH = int(os.getenv("PROCESS_WIDTH", "0"))  # 0 disables resize
JPEG_QUALITY = int(os.getenv("JPEG_QUALITY", "95"))
DEFAULT_CALIBRATION_DISTANCE = 100.0
# Hue is not meaningful for nearly gray pixels. This prevents the broad,
# low-saturation calibration range from selecting wall and furniture details.
BLUE_MARKER_MIN_SATURATION = int(os.getenv("BLUE_MARKER_MIN_SATURATION", "36"))

# Blue is used only for the four calibration markers. Adjust this with
# colorPicker.py if the actual markers are a different shade of blue.
BLUE_MARKER_RANGES = [
    # # Tuned around the blue marker center: HSV (103, 167, 127).
    # # This excludes the dark-blue chair center: HSV (109, 189, 104).
    # ((98, 125, 115), (106, 205, 160)),
    # # Lighter sunlit blue marker center: HSV (105, 153, 211).
    # ((97, 83, 141), (113, 223, 255)),
    # # Darker blue marker center: HSV (103, 123, 143).
    # ((95, 53, 73), (111, 193, 213)),

    # Center: HSV (103, 106, 137).
    ((95, 36, 67), (111, 176, 207)),
    # Center: HSV (103, 74, 254).
    ((95, 4, 184), (111, 144, 255)),
    # Center: HSV (103, 106, 137).
    ((95, 36, 67), (111, 176, 207)),
    
]

app = Flask(__name__)
calibration = None

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
    """Maps image points onto the real rectangle defined by blue markers."""

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

    def GetTransform(self, blue_points):
        if len(blue_points) != 4:
            return None, None

        source_points = OrderRectangleCorners(blue_points)
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

    def TryInitialize(self, blue_points):
        if self.transform is not None or len(blue_points) != 4:
            return False

        transform, ordered_corners = self.GetTransform(blue_points)
        if transform is None:
            return False

        self.transform = transform
        self.ordered_corners = ordered_corners
        self.calibrated_at = time.time()
        return True

    def GetCachedTransform(self):
        return self.transform, self.ordered_corners


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

    if ranges is BLUE_MARKER_RANGES:
        saturation_mask = cv2.inRange(
            hsv[:, :, 1],
            BLUE_MARKER_MIN_SATURATION,
            255,
        )
        mask_total = cv2.bitwise_and(mask_total, saturation_mask)

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

    transform = None
    ordered_corners = None
    status_text = "calibration: not configured"

    if calibration is not None:
        transform, ordered_corners = calibration.GetCachedTransform()

        if transform is None:
            blue_markers = FindBlobCenters(hsv, BLUE_MARKER_RANGES)
            # FindBlobCenters sorts by area, so retain the four most prominent
            # blobs when extra blue objects are visible during startup.
            calibration_markers = blue_markers[:4]
            blue_points = [(cx, cy) for cx, cy, _ in calibration_markers]

            for index, (cx, cy, _area) in enumerate(calibration_markers, start=1):
                cv2.circle(frame, (cx, cy), 12, CALIBRATION_COLOR, 2)
                cv2.putText(
                    frame,
                    f"blue {index}",
                    (cx + 15, cy),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    CALIBRATION_COLOR,
                    2,
                )

            if calibration.TryInitialize(blue_points):
                transform, ordered_corners = calibration.GetCachedTransform()
                status_text = (
                    f"calibration: saved transform from {len(blue_points)} blue markers"
                )
            else:
                status_text = (
                    f"calibration: waiting for 4 blue markers - {len(blue_points)} found"
                )
        else:
            status_text = "calibration: using saved transform - recalibrate if camera moved"

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
            calibration.width,
            calibration.height,
            calibration.width,
            calibration.height,
        )
        ordered_points = ordered_corners.astype(np.int32)
        for index, edge_length in enumerate(edge_lengths):
            point_a = ordered_points[index]
            point_b = ordered_points[(index + 1) % 4]
            midpoint = ((point_a + point_b) / 2).astype(int)
            cv2.putText(
                frame,
                f"{edge_length:g} {calibration.unit}",
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

            if calibration is not None and transform is not None and is_green_pink_pair:
                real_distance = calibration.Distance(transform, p1, p2)
                distance_label = (
                    f"{name_a}-{name_b}: {real_distance:.2f} {calibration.unit}"
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


def PromptForDistance(label, supplied_value):
    if supplied_value is not None:
        return PositiveDistance(supplied_value, label)

    while True:
        try:
            return PositiveDistance(input(f"{label} side length: "), label)
        except argparse.ArgumentTypeError as error:
            print(error)


def ConfigureCalibration():
    parser = argparse.ArgumentParser(
        description="Measure pink-green distance using four blue rectangle markers."
    )
    parser.add_argument(
        "--top",
        default=DEFAULT_CALIBRATION_DISTANCE,
        help="Top side length (default: 100)",
    )
    parser.add_argument(
        "--right",
        default=DEFAULT_CALIBRATION_DISTANCE,
        help="Right side length (default: 100)",
    )
    parser.add_argument(
        "--bottom",
        default=DEFAULT_CALIBRATION_DISTANCE,
        help="Bottom side length (default: 100)",
    )
    parser.add_argument(
        "--left",
        default=DEFAULT_CALIBRATION_DISTANCE,
        help="Left side length (default: 100)",
    )
    parser.add_argument(
        "--interactive",
        action="store_true",
        help="Prompt for all four side lengths instead of using command-line defaults",
    )
    parser.add_argument(
        "--unit",
        default=os.getenv("DISTANCE_UNIT", "cm"),
        help="Unit used for all four side lengths (default: cm)",
    )
    args = parser.parse_args()

    if args.interactive:
        print("Enter the real side lengths of the rectangle formed by the 4 blue markers.")
        print(f"All values use the same unit: {args.unit}")
        top = PromptForDistance("Top", None)
        right = PromptForDistance("Right", None)
        bottom = PromptForDistance("Bottom", None)
        left = PromptForDistance("Left", None)
    else:
        top = PromptForDistance("Top", args.top)
        right = PromptForDistance("Right", args.right)
        bottom = PromptForDistance("Bottom", args.bottom)
        left = PromptForDistance("Left", args.left)
        print(
            f"Calibration rectangle: top={top:g}, right={right:g}, "
            f"bottom={bottom:g}, left={left:g} {args.unit}"
        )

    difference_width = abs(top - bottom)
    difference_height = abs(right - left)
    if difference_width > 0.01 or difference_height > 0.01:
        print(
            "Note: opposite sides differ. Calibration uses their averages "
            "because the blue markers must describe a rectangle."
        )

    return HomographyCalibration(top, right, bottom, left, args.unit)


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
    calibration = ConfigureCalibration()
    worker.Start()
    app.run(host="0.0.0.0", port=5000, threaded=True)
