import os
import time
import math
import threading
import argparse
import json
import sys
from collections import deque
from statistics import median
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

REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from mqtt.distance_publisher import DistanceMqttPublisher

RTSP_URL = os.getenv(
    "RTSP_URL",
    "rtsp://oliversina135:12345678@10.0.7.12:554/stream1"
)

MIN_AREA = int(os.getenv("MIN_AREA", "300"))
TARGET_FPS = float(os.getenv("TARGET_FPS", "30"))
PROCESS_WIDTH = int(os.getenv("PROCESS_WIDTH", "0"))  # 0 disables resize
JPEG_QUALITY = int(os.getenv("JPEG_QUALITY", "95"))
DEFAULT_CALIBRATION_DISTANCE = 100.0
MQTT_ROBOT_ID = "Superintendent"
FILTER_WINDOW = int(os.getenv("FILTER_WINDOW", "5"))
FILTER_ALPHA = float(os.getenv("FILTER_ALPHA", "0.35"))
FILTER_MAX_JUMP_M = float(os.getenv("FILTER_MAX_JUMP_M", "0.50"))
FILTER_MAX_JUMP_REJECTS = int(os.getenv("FILTER_MAX_JUMP_REJECTS", "3"))
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
distance_filters = {}
distance_filters_lock = threading.Lock()
display_marker_pairs = None

COLORS = {
    # Bright pink marker (#e8428c) has HSV center [167, 182, 232].
    # Requiring both high saturation and high value prevents burgundy/mauve
    # clothing with a similar hue from being accepted as the marker.
    "pink": [
        ((160, 145, 200), (175, 255, 255)),
    ],
    "green": [
        ((38, 70, 70), (55, 255, 255)),
        ((38, 49, 87), (54, 189, 227))
    ],
}

COLOR_EXCLUSIONS = {
    # Dark false-positive sample (#7c1b3c), measured with colorPicker.py:
    # center [170, 199, 124], lower (162, 129, 54), upper (178, 255, 194).
    # Subtracting this range retains the brighter pink sample (#e8428c,
    # center [167, 182, 232]) while rejecting similarly hued dark objects.
    "pink": [
        ((162, 129, 54), (178, 255, 194)),
    ],
}

DRAW_COLOR = (255, 255, 255)
CALIBRATION_COLOR = (255, 0, 0)
UNIT_TO_METERS = {
    "m": 1.0,
    "meter": 1.0,
    "meters": 1.0,
    "cm": 0.01,
    "centimeter": 0.01,
    "centimeters": 0.01,
    "mm": 0.001,
    "millimeter": 0.001,
    "millimeters": 0.001,
}


def DistanceToMeters(distance, unit):
    if distance is None or unit is None:
        return None

    try:
        distance_value = float(distance)
    except (TypeError, ValueError):
        return None

    scale = UNIT_TO_METERS.get(str(unit).strip().lower())
    if scale is None:
        return None

    return distance_value * scale


def WorldPositionPayload(world_position, active_calibration):
    if world_position is None or active_calibration is None:
        return None

    x_world, y_world = world_position
    unit = active_calibration.unit

    return {
        "world_x_m": DistanceToMeters(x_world, unit),
        "world_y_m": DistanceToMeters(y_world, unit),
    }


def DistanceFromMeters(distance_m, unit):
    if distance_m is None or unit is None:
        return None

    scale = UNIT_TO_METERS.get(str(unit).strip().lower())
    if scale is None:
        return None

    return distance_m / scale


def ParseMarkerPair(value):
    separator = ":" if ":" in value else ","
    parts = [part.strip() for part in value.split(separator)]
    if len(parts) != 2 or not parts[0] or not parts[1]:
        raise argparse.ArgumentTypeError(
            "Marker pairs must use the form source:target, for example green:pink."
        )
    return parts[0], parts[1]


def NormalizeMarkerPairs(pairs):
    if pairs is None:
        return None
    return {frozenset(pair) for pair in pairs}


def MarkerPairIsEnabled(name_a, name_b, enabled_pairs):
    if enabled_pairs is None:
        return True
    return frozenset((name_a, name_b)) in enabled_pairs


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

    def ProjectPoint(self, transform, point):
        image_point = np.float32([[point]])
        world_point = cv2.perspectiveTransform(image_point, transform).reshape(2)
        return float(world_point[0]), float(world_point[1])

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


def BuildColorMask(hsv, ranges, excluded_ranges=None):
    mask_total = None

    for lower, upper in ranges:
        lower = np.array(lower, dtype=np.uint8)
        upper = np.array(upper, dtype=np.uint8)
        mask = cv2.inRange(hsv, lower, upper)
        mask_total = mask if mask_total is None else cv2.bitwise_or(mask_total, mask)

    for lower, upper in excluded_ranges or []:
        lower = np.array(lower, dtype=np.uint8)
        upper = np.array(upper, dtype=np.uint8)
        excluded_mask = cv2.inRange(hsv, lower, upper)
        mask_total = cv2.bitwise_and(mask_total, cv2.bitwise_not(excluded_mask))

    kernel = np.ones((5, 5), np.uint8)
    mask_total = cv2.erode(mask_total, kernel, iterations=1)
    mask_total = cv2.dilate(mask_total, kernel, iterations=2)
    return mask_total


def FindBlobCenters(hsv, ranges, excluded_ranges=None):
    mask_total = BuildColorMask(hsv, ranges, excluded_ranges)

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


def FindBlobCenter(hsv, ranges, excluded_ranges=None):
    centers = FindBlobCenters(hsv, ranges, excluded_ranges)
    return centers[0] if centers else None


def DetectMarkerPositions(frame):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    positions = {}

    for color_name, ranges in COLORS.items():
        result = FindBlobCenter(hsv, ranges, COLOR_EXCLUSIONS.get(color_name))

        if result is not None:
            cx, cy, area = result
            positions[color_name] = {
                "x": cx,
                "y": cy,
                "area": area,
            }

    return positions


def DistancePixels(p1, p2):
    dx = p2[0] - p1[0]
    dy = p2[1] - p1[1]
    d = math.sqrt(dx * dx + dy * dy)
    return d, dx, dy


def MeasurementKey(name_a, name_b):
    return f"{name_a}-{name_b}"


def MarkerPoint(position):
    return (int(position["x"]), int(position["y"]))


class DistanceStabilizer:
    def __init__(self, window_size, alpha, max_jump_m, max_jump_rejects):
        self.samples = deque(maxlen=max(1, window_size))
        self.alpha = max(0.0, min(1.0, alpha))
        self.max_jump_m = max_jump_m
        self.max_jump_rejects = max(1, max_jump_rejects)
        self.filtered_m = None
        self.rejected_jump_count = 0
        self.last_update_reset_filter = False

    def Reset(self, raw_distance_m):
        self.samples.clear()
        self.samples.append(raw_distance_m)
        self.filtered_m = raw_distance_m
        self.rejected_jump_count = 0
        self.last_update_reset_filter = True
        return self.filtered_m

    def Update(self, raw_distance_m):
        self.last_update_reset_filter = False
        if raw_distance_m is None:
            return self.filtered_m

        if (
            self.filtered_m is not None
            and self.max_jump_m > 0.0
            and abs(raw_distance_m - self.filtered_m) > self.max_jump_m
        ):
            self.rejected_jump_count += 1
            if self.rejected_jump_count >= self.max_jump_rejects:
                return self.Reset(raw_distance_m)
            return self.filtered_m

        self.rejected_jump_count = 0
        self.samples.append(raw_distance_m)
        median_distance_m = median(self.samples)

        if self.filtered_m is None:
            self.filtered_m = median_distance_m
        else:
            self.filtered_m = (
                self.alpha * median_distance_m
                + (1.0 - self.alpha) * self.filtered_m
            )

        return self.filtered_m


def StabilizeDistance(key, raw_distance_m):
    with distance_filters_lock:
        stabilizer = distance_filters.get(key)
        if stabilizer is None:
            stabilizer = DistanceStabilizer(
                FILTER_WINDOW,
                FILTER_ALPHA,
                FILTER_MAX_JUMP_M,
                FILTER_MAX_JUMP_REJECTS,
            )
            distance_filters[key] = stabilizer

        distance_m = stabilizer.Update(raw_distance_m)
        return {
            "distance_m": distance_m,
            "reset_filter": stabilizer.last_update_reset_filter,
            "rejected_jump_count": stabilizer.rejected_jump_count,
        }


def ClearDistanceFilters():
    with distance_filters_lock:
        distance_filters.clear()


def BuildDistanceMeasurements(positions, active_calibration):
    measurements = {}
    names = sorted(positions.keys())
    transform = None
    unit = None

    if active_calibration is not None:
        transform, _ = active_calibration.GetCachedTransform()
        unit = active_calibration.unit

    for i in range(len(names)):
        for j in range(i + 1, len(names)):
            name_a = names[i]
            name_b = names[j]
            p1 = MarkerPoint(positions[name_a])
            p2 = MarkerPoint(positions[name_b])
            pixel_distance, dx_pixels, dy_pixels = DistancePixels(p1, p2)
            raw_distance = None
            raw_distance_m = None
            distance_m = None
            distance = None

            if transform is not None:
                raw_distance = active_calibration.Distance(transform, p1, p2)
                raw_distance_m = DistanceToMeters(raw_distance, unit)
                world_p1 = active_calibration.ProjectPoint(transform, p1)
                world_p2 = active_calibration.ProjectPoint(transform, p2)
                dx_world = world_p2[0] - world_p1[0]
                dy_world = world_p2[1] - world_p1[1]
                dx_m = DistanceToMeters(dx_world, unit)
                dy_m = DistanceToMeters(dy_world, unit)
                stabilization = StabilizeDistance(
                    MeasurementKey(name_a, name_b),
                    raw_distance_m,
                )
                distance_m = stabilization["distance_m"]
                distance = DistanceFromMeters(distance_m, unit)
            else:
                dx_m = None
                dy_m = None
                stabilization = {
                    "reset_filter": False,
                    "rejected_jump_count": 0,
                }

            payload = {
                "from": name_a,
                "to": name_b,
                "pixel_distance": pixel_distance,
                "dx_pixels": dx_pixels,
                "dy_pixels": dy_pixels,
                "dx_m": dx_m,
                "dy_m": dy_m,
                "unit": unit,
                "distance": distance,
                "distance_m": distance_m,
                "raw_distance": raw_distance,
                "raw_distance_m": raw_distance_m,
                "filtered": distance_m is not None,
                "filter_reset": stabilization["reset_filter"],
                "filter_rejected_jump_count": stabilization["rejected_jump_count"],
            }

            measurements[MeasurementKey(name_a, name_b)] = payload
            reverse_payload = dict(payload)
            reverse_payload["from"] = name_b
            reverse_payload["to"] = name_a
            reverse_payload["dx_pixels"] = -dx_pixels
            reverse_payload["dy_pixels"] = -dy_pixels
            reverse_payload["dx_m"] = None if dx_m is None else -dx_m
            reverse_payload["dy_m"] = None if dy_m is None else -dy_m
            reverse_stabilization = StabilizeDistance(
                MeasurementKey(name_b, name_a),
                raw_distance_m,
            )
            reverse_payload["distance_m"] = reverse_stabilization["distance_m"]
            reverse_payload["distance"] = DistanceFromMeters(
                reverse_payload["distance_m"],
                unit,
            )
            reverse_payload["filter_reset"] = reverse_stabilization["reset_filter"]
            reverse_payload["filter_rejected_jump_count"] = reverse_stabilization[
                "rejected_jump_count"
            ]
            measurements[MeasurementKey(name_b, name_a)] = reverse_payload

    return measurements


def BuildWorldPositions(positions, active_calibration):
    if active_calibration is None:
        return {}

    transform, _ = active_calibration.GetCachedTransform()
    if transform is None:
        return {}

    world_positions = {}
    for name, position in positions.items():
        marker_point = MarkerPoint(position)
        world_positions[name] = WorldPositionPayload(
            active_calibration.ProjectPoint(transform, marker_point),
            active_calibration,
        )
    return world_positions


def GetMeasurementSnapshot():
    frame = grabber.GetLatest()
    if frame is None:
        return None

    frame = ResizeForProcessing(frame)
    positions = DetectMarkerPositions(frame)

    with calibration_lock:
        active_calibration = calibration

    measurements = BuildDistanceMeasurements(positions, active_calibration)
    world_positions = BuildWorldPositions(positions, active_calibration)
    return {
        "timestamp": time.time(),
        "calibrated": active_calibration is not None
        and active_calibration.transform is not None,
        "unit": active_calibration.unit if active_calibration is not None else None,
        "positions": positions,
        "world_positions": world_positions,
        "distances": measurements,
    }


def BuildDistanceEventPayload(robot_id, marker_a, marker_b):
    snapshot = GetMeasurementSnapshot()
    now = time.time()

    if snapshot is None:
        return {
            "type": "EVENT",
            "robot_id": robot_id,
            "event": "GLOBAL_SEARCH_MARKER_DISTANCE_UNAVAILABLE",
            "state": "MEASURING",
            "timestamp": now,
            "source_marker": marker_a,
            "target_marker": marker_b,
            "error": "No frame is available yet.",
        }

    measurement = snapshot["distances"].get(MeasurementKey(marker_a, marker_b))
    if measurement is None:
        return {
            "type": "EVENT",
            "robot_id": robot_id,
            "event": "GLOBAL_SEARCH_MARKER_DISTANCE_UNAVAILABLE",
            "state": "MEASURING",
            "timestamp": now,
            "source_marker": marker_a,
            "target_marker": marker_b,
            "available_markers": sorted(snapshot["positions"].keys()),
            "error": "Could not detect both requested markers.",
        }

    return {
        "type": "EVENT",
        "robot_id": robot_id,
        "event": "GLOBAL_SEARCH_MARKER_DISTANCE",
        "state": "MEASURING",
        "timestamp": now,
        "source_marker": marker_a,
        "target_marker": marker_b,
        "distance_m": measurement.get("distance_m"),
        "raw_distance_m": measurement.get("raw_distance_m"),
        "dx_m": measurement.get("dx_m"),
        "dy_m": measurement.get("dy_m"),
        "measurement_timestamp": snapshot["timestamp"],
    }


def BuildDistanceEventPayloads(robot_id, marker_pairs):
    return [
        BuildDistanceEventPayload(robot_id, marker_a, marker_b)
        for marker_a, marker_b in marker_pairs
    ]


def ProcessFrame(frame):
    detected_positions = DetectMarkerPositions(frame)

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

    for color_name, position in detected_positions.items():
        cx = int(position["x"])
        cy = int(position["y"])
        area = float(position["area"])

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

    measurements = BuildDistanceMeasurements(detected_positions, active_calibration)
    names = sorted(detected_positions.keys())
    y_text = 60

    for i in range(len(names)):
        for j in range(i + 1, len(names)):
            name_a = names[i]
            name_b = names[j]
            p1 = MarkerPoint(detected_positions[name_a])
            p2 = MarkerPoint(detected_positions[name_b])
            marker_pair_enabled = MarkerPairIsEnabled(
                name_a,
                name_b,
                display_marker_pairs,
            )
            if not marker_pair_enabled:
                continue

            cv2.line(frame, p1, p2, DRAW_COLOR, 2)
            mid_x = int((p1[0] + p2[0]) / 2)
            mid_y = int((p1[1] + p2[1]) / 2)

            measurement = measurements.get(MeasurementKey(name_a, name_b))
            if measurement and measurement["distance"] is not None:
                filter_note = " reset" if measurement.get("filter_reset") else ""
                distance_label = (
                    f"{name_a}-{name_b}: "
                    f"raw {measurement['raw_distance']:.2f} {measurement['unit']} | "
                    f"stable {measurement['distance']:.2f} {measurement['unit']}{filter_note}"
                )
            else:
                distance_label = f"{name_a}-{name_b}: calibration needed"

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
        ClearDistanceFilters()

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
        ClearDistanceFilters()
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
        ClearDistanceFilters()
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
            <h2>Pair Distances</h2>
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


@app.get("/measurements")
def measurements():
    snapshot = GetMeasurementSnapshot()
    if snapshot is None:
        return jsonify(error="No frame is available yet."), 503
    return jsonify(snapshot)


@app.get("/distance")
def distance():
    marker_a = request.args.get("from", "").strip()
    marker_b = request.args.get("to", "").strip()

    if not marker_a or not marker_b:
        return jsonify(error="Query parameters 'from' and 'to' are required."), 400

    snapshot = GetMeasurementSnapshot()
    if snapshot is None:
        return jsonify(error="No frame is available yet."), 503

    measurement = snapshot["distances"].get(MeasurementKey(marker_a, marker_b))
    if measurement is None:
        return jsonify(
            error="Could not detect both requested markers.",
            available_markers=sorted(snapshot["positions"].keys()),
        ), 404

    payload = dict(measurement)
    payload["timestamp"] = snapshot["timestamp"]
    payload["calibrated"] = snapshot["calibrated"]
    return jsonify(payload)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Measure pair distances using a saved click calibration."
    )
    parser.add_argument(
        "--calibrate",
        action="store_true",
        help="Select four new corners and enter their measured side lengths.",
    )
    parser.add_argument(
        "--mqtt-broker",
        help="MQTT broker IP/host. Enables distance publication when set.",
    )
    parser.add_argument(
        "--mqtt-port",
        type=int,
        default=1883,
        help="MQTT broker port.",
    )
    parser.add_argument(
        "--mqtt-marker-pair",
        action="append",
        type=ParseMarkerPair,
        help="Marker pair to publish, for example green:pink. Can be used more than once.",
    )
    parser.add_argument(
        "--mqtt-period",
        type=float,
        default=0.5,
        help="Seconds between MQTT distance events.",
    )
    parser.add_argument(
        "--filter-window",
        type=int,
        default=FILTER_WINDOW,
        help="Rolling median sample count for distance stabilization.",
    )
    parser.add_argument(
        "--filter-alpha",
        type=float,
        default=FILTER_ALPHA,
        help="EMA alpha for distance stabilization, from 0.0 to 1.0.",
    )
    parser.add_argument(
        "--filter-max-jump-m",
        type=float,
        default=FILTER_MAX_JUMP_M,
        help="Ignore single-sample jumps larger than this many meters. Use 0 to disable.",
    )
    parser.add_argument(
        "--filter-max-jump-rejects",
        type=int,
        default=FILTER_MAX_JUMP_REJECTS,
        help="Reset stabilization after this many consecutive large jumps.",
    )
    parser.add_argument(
        "--display-pair",
        action="append",
        type=ParseMarkerPair,
        help="Marker pair to draw in the video, for example green:pink. Can be used more than once. Defaults to all detected pairs.",
    )
    args = parser.parse_args()

    FILTER_WINDOW = max(1, args.filter_window)
    FILTER_ALPHA = max(0.0, min(1.0, args.filter_alpha))
    FILTER_MAX_JUMP_M = max(0.0, args.filter_max_jump_m)
    FILTER_MAX_JUMP_REJECTS = max(1, args.filter_max_jump_rejects)
    display_marker_pairs = NormalizeMarkerPairs(args.display_pair)

    mqtt_marker_pairs = args.mqtt_marker_pair or []
    if (args.mqtt_broker is not None or mqtt_marker_pairs) and (
        args.mqtt_broker is None or not mqtt_marker_pairs
    ):
        parser.error(
            "--mqtt-broker and at least one --mqtt-marker-pair must be used together."
        )

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
    mqtt_publisher = None
    if args.mqtt_broker is not None:
        mqtt_publisher = DistanceMqttPublisher(
            broker=args.mqtt_broker,
            port=args.mqtt_port,
            robot_id=MQTT_ROBOT_ID,
            period=args.mqtt_period,
            payload_provider=lambda: BuildDistanceEventPayloads(
                MQTT_ROBOT_ID,
                mqtt_marker_pairs,
            ),
        )
        mqtt_publisher.Start()
        print(
            "Publishing MQTT distance pairs:",
            mqtt_marker_pairs,
            "on",
            mqtt_publisher.topic,
        )

    try:
        app.run(host="0.0.0.0", port=5000, threaded=True)
    finally:
        if mqtt_publisher is not None:
            mqtt_publisher.Stop()
