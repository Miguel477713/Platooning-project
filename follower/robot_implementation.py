import os
import sys
import time
from pathlib import Path
from typing import Optional

import cv2
import numpy as np

from common.models import DetectionResult, SuperintendentMeasurement


CENTER_DEADZONE_X = 0.10
CENTER_DEADZONE_Y = 0.10

TARGET_AREA_MIN = 0.05
OBSTACLE_MIN_CM = 25.0
SONIC_INTERVAL = 0.20
CAMERA_COMMAND_INTERVAL = 0.12
FRAME_TIMEOUT = 1.0

PAN_STEP_GAIN = 6
TILT_STEP_GAIN = 3
MAX_PAN_STEP = 4
MAX_TILT_STEP = 1
MOVE_SPEED = 7
SPEED_MIN = 2
SPEED_MAX = 10
MOVE_FORWARD_Y = 12
MOVE_BACKWARD_Y = -10
OVERHEAD_MOVE_GAIN = 8
OVERHEAD_MAX_STEP = 8
OVERHEAD_AXIS_DEADZONE_M = 0.08
OVERHEAD_CLOSE_DISTANCE_M = 0.40
OVERHEAD_DIRECTION_LEARN_DELTA_M = 0.02
OVERHEAD_DIRECTION_LEARN_MIN_STEP = 6
OVERHEAD_DIRECTION_LEARN_OBSERVE_S = 5.00
VISUAL_ACQUIRE_MOVE_GAIN = 5
VISUAL_ACQUIRE_MAX_STEP = 4
VISUAL_ACQUIRE_MOVE_INTERVAL = 0.35
VISUAL_ACQUIRE_PAN_STEP = 2
VISUAL_ACQUIRE_ROTATE_PAN_THRESHOLD = 26
VISUAL_ACQUIRE_ROTATE_STEP = 3
GLOBAL_SEARCH_RETURN_MOVE_INTERVAL = 0.35
GLOBAL_SEARCH_MEMORY_MAX_STEPS = 100

PAN_MAX = 35
TILT_MAX = 35
CAMERA_CENTER_X = 90
CAMERA_CENTER_Y = 75
TURN_DEADZONE_PAN = 10
TURN_GAIN = 0.15
ROTATE_TARGET_X = 0.20
BODY_Z = 20
OCCLUDED_TARGET_AREA_MAX = 0.025
OCCLUSION_DARK_RATIO = 0.30
OCCLUSION_MARGIN = 0.35


COLOR_RANGES = {
    "red": [
        ((0, 100, 80), (10, 255, 255)),
        ((170, 100, 80), (180, 255, 255)),
    ],
    "green": [
        ((35, 70, 60), (85, 255, 255)),
    ],
    "blue": [
        ((90, 70, 60), (130, 255, 255)),
    ],
    "yellow": [
        ((20, 80, 80), (35, 255, 255)),
    ],
}


def clamp(value, minimum, maximum):
    return max(minimum, min(maximum, value))


def signed_step(value, gain, max_step):
    step = int(round(abs(value) * gain))
    step = clamp(step, 1, max_step)
    return step if value > 0 else -step


def overhead_axis_step(value, gain, max_step, learned):
    step = int(round(abs(value) * gain))
    min_step = 1 if learned else min(OVERHEAD_DIRECTION_LEARN_MIN_STEP, max_step)
    step = clamp(step, min_step, max_step)
    return step if value > 0 else -step


class FollowerRobotImplementation:
    """Hardware/vision implementation layer.
    """

    # =====================================================
    # Vision placeholders
    # =====================================================

    def global_detect(self, target_color: str) -> DetectionResult:
        """Full-frame/global blob detection.

        Replace with real camera/blob detection code.
        """
        print(f"[VISION] global search for color={target_color}")
        return DetectionResult(detected=False)

    def local_detect(self, target_color: str) -> DetectionResult:
        """Local/ROI blob tracking.

        Replace with real camera/blob tracking code.
        """
        print(f"[VISION] local search for color={target_color}")
        return DetectionResult(detected=False)

    # =====================================================
    # Motor placeholders
    # =====================================================

    def stop_motors(self) -> None:
        print("[MOTOR] stop")

    def global_search_stop(self) -> None:
        print("[MOTOR] global search stop")

    def global_search_guided_motion(self, measurement: SuperintendentMeasurement) -> None:
        print(
            "[MOTOR] global search guided motion "
            f"source={measurement.source_marker}, target={measurement.target_marker}, "
            f"distance={measurement.distance_m}, dx={measurement.dx_m}, dy={measurement.dy_m}"
        )

    def global_search_return_motion(self) -> bool:
        print("[MOTOR] global search return motion unavailable")
        return False

    def clear_global_search_memory(self) -> None:
        pass

    def global_visual_acquire_motion(self, measurement: SuperintendentMeasurement) -> None:
        print(
            "[MOTOR] global visual acquire motion "
            f"source={measurement.source_marker}, target={measurement.target_marker}, "
            f"distance={measurement.distance_m}, dx={measurement.dx_m}, dy={measurement.dy_m}"
        )

    def global_approach_motion(self, *, target_id: str, distance_m: float, goal_m: float) -> None:
        print(
            "[MOTOR] global approach motion "
            f"target={target_id}, distance={distance_m}, goal={goal_m}"
        )

    def local_lock_motion(self, result: DetectionResult) -> None:
        print("[MOTOR] local lock alignment")

    def local_follow_motion(self, result: DetectionResult, desired_gap_m: float) -> None:
        error_m = None
        if result.distance_m is not None:
            error_m = result.distance_m - desired_gap_m

        print(
            "[MOTOR] local follow motion "
            f"distance={result.distance_m}, desired_gap={desired_gap_m}, error={error_m}"
        )


class FreenoveDirectRobotImplementation(FollowerRobotImplementation):
    """Freenove hardware implementation without the Freenove TCP server.
    """

    def __init__(
        self,
        *,
        speed: int = MOVE_SPEED,
        target_area: float = TARGET_AREA_MIN,
        pan_max: int = PAN_MAX,
        tilt_max: int = TILT_MAX,
        camera_only: bool = False,
        invert_pan: bool = False,
        invert_tilt: bool = False,
        lock_tilt: bool = False,
        move_only_when_centered: bool = True,
        wake_robot: bool = True,
        enable_turning: bool = True,
        camera_center_y: int = CAMERA_CENTER_Y,
        ignore_lower_frame: float = 0.25,
        body_z: int = BODY_Z,
        occluded_target_area_max: float = OCCLUDED_TARGET_AREA_MAX,
        occlusion_dark_ratio: float = OCCLUSION_DARK_RATIO,
        show_video: bool = False,
    ):
        self.speed = clamp(int(speed), SPEED_MIN, SPEED_MAX)
        self.target_area_min = clamp(target_area, 0.01, 0.90)
        self.target_area_max = self.target_area_min + 0.15
        self.camera_only = camera_only
        self.invert_pan = invert_pan
        self.invert_tilt = invert_tilt
        self.lock_tilt = lock_tilt
        self.move_only_when_centered = move_only_when_centered
        self.wake_robot = wake_robot
        self.enable_turning = enable_turning
        self.camera_center_y = camera_center_y
        self.ignore_lower_frame = clamp(ignore_lower_frame, 0.0, 0.8)
        self.body_z = clamp(int(body_z), -20, 20)
        self.occluded_target_area_max = clamp(occluded_target_area_max, 0.001, 0.20)
        self.occlusion_dark_ratio = clamp(occlusion_dark_ratio, 0.05, 0.90)
        self.show_video = show_video
        self.video_window_name = "Follower Direct Camera"
        self.pan_max = clamp(int(pan_max), 10, 60)
        self.tilt_max = clamp(int(tilt_max), 10, 60)

        self.pan_angle = 0
        self.tilt_angle = 0
        self.target_x = 0.0
        self.target_y = 0.0
        self.target_area = 0.0
        self.target_occluded = False
        self.action_status = "idle"
        self.last_seen_time = 0.0
        self.search_direction = 1
        self.last_sonic_request = 0.0
        self.obstacle_distance: Optional[float] = None
        self.last_camera_command = 0.0
        self.last_camera_x = None
        self.last_camera_y = None
        self.last_frame_time = 0.0
        self.visual_acquire_pan_direction = 1
        self.last_visual_acquire_move = 0.0
        self.global_search_move_history = []
        self.last_global_search_return_move = 0.0
        self.overhead_last_grid_axis = "y"
        self.overhead_axis_sign = {"x": 1, "y": 1}
        self.overhead_axis_learned = {"x": False, "y": False}
        self.overhead_axis_probe = None

        self.server_dir = self._prepare_freenove_imports()
        self.control = None
        self.camera = None
        self.ultrasonic = None
        self._initialize_hardware()

    def _prepare_freenove_imports(self) -> Path:
        repo_root = Path(__file__).resolve().parents[1]
        server_dir = repo_root / "Freenove_Big_Hexapod_Robot_Kit_for_Raspberry_Pi" / "Code" / "Server"
        if not server_dir.exists():
            raise FileNotFoundError(f"Freenove Server directory not found: {server_dir}")

        server_path = str(server_dir)
        if server_path not in sys.path:
            sys.path.insert(0, server_path)
        return server_dir

    def _initialize_hardware(self) -> None:
        previous_cwd = os.getcwd()
        try:
            os.chdir(self.server_dir)
            from camera import Camera
            from control import Control
            from ultrasonic import Ultrasonic

            self.control = Control()
            self.control.condition_thread.daemon = True
            self.control.condition_thread.start()
            self.camera = Camera()
            self.camera.start_stream()
            self.ultrasonic = Ultrasonic()
        except Exception as exc:
            raise RuntimeError(
                "Could not initialize Freenove direct hardware. Run this on the "
                "Raspberry Pi with the Freenove hardware libraries installed."
            ) from exc
        finally:
            os.chdir(previous_cwd)

        if self.wake_robot:
            self.stand_up()
        self.reset_camera()
        self.stop_motors()
        self.last_seen_time = time.time()

    def global_detect(self, target_color: str) -> DetectionResult:
        return self._detect(target_color)

    def local_detect(self, target_color: str) -> DetectionResult:
        return self._detect(target_color)

    #Does not move camera
    def _detect(self, target_color: str) -> DetectionResult:
        frame = self.get_latest_frame()
        if frame is None:
            if time.time() - self.last_frame_time > FRAME_TIMEOUT:
                self.stop_motors()
            return DetectionResult(detected=False)

        detected, x, y, area, occluded, debug = self.detect_target(frame, target_color)
        distance_cm = self.request_ultrasonic_distance()
        distance_m = None if distance_cm is None else distance_cm / 100.0

        if not detected:
            self.target_occluded = False
            self.show_debug_frame(frame, debug, target_color, False, distance_m)
            return DetectionResult(detected=False, distance_m=distance_m)

        self.target_x = x
        self.target_y = y
        self.target_area = area
        self.target_occluded = occluded
        self.last_seen_time = time.time()
        self.show_debug_frame(frame, debug, target_color, True, distance_m)

        confidence = clamp(area / max(self.target_area_min, 0.001), 0.0, 1.0)
        bearing_deg = x * 45.0
        return DetectionResult(
            detected=True,
            distance_m=distance_m,
            bearing_deg=bearing_deg,
            confidence=confidence,
            target_x=x,
            target_y=y,
            target_area=area,
            occluded=occluded,
        )

    def get_latest_frame(self):
        if self.camera is None:
            return None

        with self.camera.streaming_output.condition:
            available = self.camera.streaming_output.condition.wait(timeout=FRAME_TIMEOUT)
            if not available or self.camera.streaming_output.frame is None:
                return None
            jpg = self.camera.streaming_output.frame

        frame = cv2.imdecode(np.frombuffer(jpg, dtype=np.uint8), cv2.IMREAD_COLOR)
        if frame is not None:
            self.last_frame_time = time.time()
        return frame

    def detect_target(self, frame, target_color: str):
        if target_color not in COLOR_RANGES:
            return False, 0.0, 0.0, 0.0, False, {"mask": None}

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = np.zeros(hsv.shape[:2], dtype=np.uint8)

        for lower, upper in COLOR_RANGES[target_color]:
            lower_bound = np.array(lower, dtype=np.uint8)
            upper_bound = np.array(upper, dtype=np.uint8)
            mask = cv2.bitwise_or(mask, cv2.inRange(hsv, lower_bound, upper_bound))

        if self.ignore_lower_frame > 0.0:
            height = mask.shape[0]
            cutoff = int(height * (1.0 - self.ignore_lower_frame))
            mask[cutoff:height, :] = 0

        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        contours = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        contours = contours[0] if len(contours) == 2 else contours[1]
        if len(contours) == 0:
            return False, 0.0, 0.0, 0.0, False, {"mask": mask}

        contour = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(contour)
        height, width = frame.shape[:2]
        relative_area = area / float(width * height)
        if relative_area < 0.002:
            return False, 0.0, 0.0, 0.0, False, {"mask": mask}

        moments = cv2.moments(contour)
        if moments["m00"] == 0:
            return False, 0.0, 0.0, 0.0, False, {"mask": mask}

        center_x = int(moments["m10"] / moments["m00"])
        center_y = int(moments["m01"] / moments["m00"])
        target_x = (center_x - width / 2.0) / (width / 2.0)
        target_y = (center_y - height / 2.0) / (height / 2.0)

        x, y, w, h = cv2.boundingRect(contour)
        occluded, _ = self.detect_target_occlusion(frame, mask, x, y, w, h, relative_area)
        debug = {
            "mask": mask,
            "box": (x, y, w, h),
            "center": (center_x, center_y),
        }
        return True, target_x, target_y, relative_area, occluded, debug

    def show_debug_frame(self, frame, debug, target_color: str, detected: bool, distance_m) -> None:
        if not self.show_video:
            return

        annotated = frame.copy()
        height, width = annotated.shape[:2]
        cv2.line(annotated, (width // 2, 0), (width // 2, height), (255, 255, 255), 1)
        cv2.line(annotated, (0, height // 2), (width, height // 2), (255, 255, 255), 1)

        if detected and debug.get("box") is not None:
            x, y, w, h = debug["box"]
            center_x, center_y = debug["center"]
            cv2.rectangle(annotated, (x, y), (x + w, y + h), (0, 255, 0), 2)
            cv2.circle(annotated, (center_x, center_y), 4, (255, 0, 0), -1)

        status = (
            f"color={target_color} detected={detected} "
            f"x={self.target_x:.2f} y={self.target_y:.2f} area={self.target_area:.3f} "
            f"pan={self.pan_angle} tilt={self.tilt_angle} "
            f"dist={'--' if distance_m is None else round(distance_m, 2)} "
            f"{self.action_status}"
        )
        cv2.putText(
            annotated,
            status,
            (10, 24),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.45,
            (0, 255, 255),
            1,
            cv2.LINE_AA,
        )

        mask = debug.get("mask")
        if mask is not None:
            mask_bgr = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
            mask_bgr = cv2.resize(mask_bgr, (width, height))
            annotated = np.hstack((annotated, mask_bgr))

        cv2.imshow(self.video_window_name, annotated)
        cv2.waitKey(1)

    def detect_target_occlusion(self, frame, target_mask, x, y, w, h, relative_area):
        if relative_area > self.occluded_target_area_max:
            return False, 0.0

        height, width = frame.shape[:2]
        margin_x = int(max(w, 1) * OCCLUSION_MARGIN)
        margin_y = int(max(h, 1) * OCCLUSION_MARGIN)
        x0 = clamp(x - margin_x, 0, width - 1)
        y0 = clamp(y - margin_y, 0, height - 1)
        x1 = clamp(x + w + margin_x, x0 + 1, width)
        y1 = clamp(y + h + margin_y, y0 + 1, height)

        region = frame[y0:y1, x0:x1]
        region_target = target_mask[y0:y1, x0:x1]
        gray = cv2.cvtColor(region, cv2.COLOR_BGR2GRAY)
        dark_mask = cv2.inRange(gray, 0, 80)
        dark_mask[region_target > 0] = 0
        dark_ratio = cv2.countNonZero(dark_mask) / float(dark_mask.size)
        return dark_ratio >= self.occlusion_dark_ratio, dark_ratio

    def stop_motors(self) -> None:
        self.send_move(0, 0)

    def global_search_stop(self) -> None:
        self.action_status = "global-search-idle"
        self.stop_motors()

    def global_search_guided_motion(self, measurement: SuperintendentMeasurement) -> None:
        if self.camera_only:
            self.action_status = "overhead-camera-only"
            self.stop_motors()
            return

        distance_m = measurement.distance_m
        dx_m = measurement.dx_m
        dy_m = measurement.dy_m

        if distance_m is None or dx_m is None or dy_m is None:
            self.action_status = "overhead-missing"
            self.stop_motors()
            return

        distance_cm = self.request_ultrasonic_distance()
        if distance_cm is not None and distance_cm < OBSTACLE_MIN_CM:
            self.action_status = "overhead-obstacle-back"
            self.send_move_xy(0, MOVE_BACKWARD_Y, 0)
            return

        if distance_m <= OVERHEAD_CLOSE_DISTANCE_M:
            self.action_status = "overhead-close-hold"
            self.stop_motors()
            return

        x, y = self.overhead_grid_step(
            dx_m,
            dy_m,
            distance_m=distance_m,
            measurement_timestamp=measurement.timestamp,
            gain=OVERHEAD_MOVE_GAIN,
            max_step=OVERHEAD_MAX_STEP,
        )

        if x == 0 and y == 0:
            self.action_status = "overhead-deadzone"
            self.stop_motors()
            return

        self.action_status = "overhead-guided"
        self.send_move_xy(x, y, 0)
        self.remember_global_search_move(x, y, 0)

    def global_visual_acquire_motion(self, measurement: SuperintendentMeasurement) -> None:
        if self.camera_only:
            self.action_status = "visual-acquire-camera-only"
            self.visual_acquire_camera_sweep()
            self.stop_motors()
            return

        self.visual_acquire_camera_sweep()

        distance_m = measurement.distance_m
        dx_m = measurement.dx_m
        dy_m = measurement.dy_m

        if distance_m is None or dx_m is None or dy_m is None:
            self.action_status = "visual-acquire-missing"
            self.stop_motors()
            return

        distance_cm = self.request_ultrasonic_distance()
        if distance_cm is not None and distance_cm < OBSTACLE_MIN_CM:
            self.action_status = "visual-acquire-obstacle-back"
            self.send_move_xy(0, MOVE_BACKWARD_Y, 0)
            return

        now = time.time()
        if now - self.last_visual_acquire_move < VISUAL_ACQUIRE_MOVE_INTERVAL:
            self.action_status = "visual-acquire-settle"
            self.stop_motors()
            return

        turn = self.visual_acquire_turn_from_pan()
        if turn != 0:
            self.last_visual_acquire_move = now
            self.action_status = "visual-acquire-rotate"
            self.send_move_xy(0, 0, turn)
            return

        if distance_m <= OVERHEAD_CLOSE_DISTANCE_M:
            self.action_status = "visual-acquire-hold"
            self.stop_motors()
            return

        x, y = self.overhead_grid_step(
            dx_m,
            dy_m,
            distance_m=distance_m,
            measurement_timestamp=measurement.timestamp,
            gain=VISUAL_ACQUIRE_MOVE_GAIN,
            max_step=VISUAL_ACQUIRE_MAX_STEP,
        )

        self.last_visual_acquire_move = now

        if x == 0 and y == 0:
            self.action_status = "visual-acquire-hold"
            self.stop_motors()
            return

        self.action_status = "visual-acquire-creep"
        self.send_move_xy(x, y, 0)
        self.remember_global_search_move(x, y, 0)

    def overhead_grid_step(self, dx_m, dy_m, *, distance_m, measurement_timestamp, gain, max_step):
        self.update_overhead_direction_learning(dx_m, dy_m, distance_m, measurement_timestamp)
        if self.overhead_axis_probe is not None:
            axis = self.overhead_axis_probe["axis"]
            if self.overhead_probe_active():
                self.action_status = f"overhead-{axis}-learn-probe"
            else:
                self.action_status = f"overhead-{axis}-learn-wait"
            return self.overhead_probe_motion()

        x_ready = abs(dx_m) > OVERHEAD_AXIS_DEADZONE_M
        y_ready = abs(dy_m) > OVERHEAD_AXIS_DEADZONE_M

        if not x_ready and not y_ready:
            return 0, 0

        if x_ready and y_ready:
            axis = "x" if self.overhead_last_grid_axis == "y" else "y"
        else:
            axis = "x" if x_ready else "y"

        self.overhead_last_grid_axis = axis
        if axis == "x":
            raw_x = overhead_axis_step(
                dx_m, gain, max_step, self.overhead_axis_learned["x"]
            )
            x = raw_x * self.overhead_axis_sign["x"]
            self.start_overhead_direction_probe("x", dx_m, distance_m, measurement_timestamp, x)
            return x, 0

        raw_y = overhead_axis_step(
            dy_m, gain, max_step, self.overhead_axis_learned["y"]
        )
        y = raw_y * self.overhead_axis_sign["y"]
        self.start_overhead_direction_probe("y", dy_m, distance_m, measurement_timestamp, y)
        return 0, y

    def overhead_probe_motion(self):
        if not self.overhead_probe_active():
            return 0, 0

        axis = self.overhead_axis_probe["axis"]
        command = self.overhead_axis_probe["command"]
        if axis == "x":
            return command, 0
        return 0, command

    def overhead_probe_active(self):
        elapsed_s = time.time() - self.overhead_axis_probe["start_time"]
        return elapsed_s < OVERHEAD_DIRECTION_LEARN_OBSERVE_S

    def start_overhead_direction_probe(
        self, axis, error_m, distance_m, measurement_timestamp, command
    ) -> None:
        if self.overhead_axis_learned[axis] or self.overhead_axis_probe is not None or command == 0:
            return

        self.overhead_axis_probe = {
            "axis": axis,
            "error_m": error_m,
            "distance_m": distance_m,
            "timestamp": measurement_timestamp,
            "command": command,
            "start_time": time.time(),
        }
        self.action_status = f"overhead-{axis}-learn-probe"

    def update_overhead_direction_learning(self, dx_m, dy_m, distance_m, measurement_timestamp) -> None:
        if self.overhead_axis_probe is None:
            return
        axis = self.overhead_axis_probe["axis"]
        if self.overhead_axis_learned[axis]:
            self.overhead_axis_probe = None
            return
        if time.time() - self.overhead_axis_probe["start_time"] < OVERHEAD_DIRECTION_LEARN_OBSERVE_S:
            return
        if measurement_timestamp <= self.overhead_axis_probe["timestamp"]:
            return

        before_error = abs(self.overhead_axis_probe["error_m"])
        after_error = abs(dx_m if axis == "x" else dy_m)
        before_distance = self.overhead_axis_probe["distance_m"]

        axis_error_worse = after_error > before_error + OVERHEAD_DIRECTION_LEARN_DELTA_M
        distance_worse = distance_m > before_distance + OVERHEAD_DIRECTION_LEARN_DELTA_M

        if axis_error_worse or distance_worse:
            self.overhead_axis_sign[axis] *= -1

        self.overhead_axis_learned[axis] = True
        self.overhead_axis_probe = None

    def remember_global_search_move(self, x, y, angle=0):
        if x == 0 and y == 0 and angle == 0:
            return

        self.global_search_move_history.append((int(x), int(y), int(angle)))
        if len(self.global_search_move_history) > GLOBAL_SEARCH_MEMORY_MAX_STEPS:
            self.global_search_move_history.pop(0)

    def global_search_return_motion(self) -> bool:
        if self.camera_only:
            self.action_status = "return-camera-only"
            self.stop_motors()
            return False

        now = time.time()
        if now - self.last_global_search_return_move < GLOBAL_SEARCH_RETURN_MOVE_INTERVAL:
            self.action_status = "return-settle"
            self.stop_motors()
            return True

        while self.global_search_move_history:
            x, y, angle = self.global_search_move_history.pop()
            if x == 0 and y == 0 and angle == 0:
                continue

            distance_cm = self.request_ultrasonic_distance()
            if distance_cm is not None and distance_cm < OBSTACLE_MIN_CM:
                self.action_status = "return-obstacle-back"
                self.send_move_xy(0, MOVE_BACKWARD_Y, 0)
                return True

            self.last_global_search_return_move = now
            self.action_status = "return-memory"
            self.send_move_xy(-x, -y, -angle)
            return True

        self.action_status = "return-empty"
        self.stop_motors()
        return False

    def clear_global_search_memory(self) -> None:
        self.global_search_move_history = []
        self.last_global_search_return_move = 0.0
        self.clear_overhead_feedback()

    def clear_overhead_feedback(self) -> None:
        self.overhead_last_grid_axis = "y"
        self.overhead_axis_probe = None

    def visual_acquire_camera_sweep(self):
        self.pan_angle += self.visual_acquire_pan_direction * VISUAL_ACQUIRE_PAN_STEP
        if self.pan_angle > self.pan_max:
            self.pan_angle = self.pan_max
            self.visual_acquire_pan_direction = -1
        elif self.pan_angle < -self.pan_max:
            self.pan_angle = -self.pan_max
            self.visual_acquire_pan_direction = 1

        self.send_camera()

    def visual_acquire_turn_from_pan(self):
        if not self.enable_turning:
            return 0

        pan_threshold = min(VISUAL_ACQUIRE_ROTATE_PAN_THRESHOLD, self.pan_max)
        if abs(self.pan_angle) < pan_threshold:
            return 0

        turn = -VISUAL_ACQUIRE_ROTATE_STEP if self.pan_angle > 0 else VISUAL_ACQUIRE_ROTATE_STEP
        return turn

    def global_approach_motion(self, *, target_id: str, distance_m: float, goal_m: float) -> None:
        self.center_camera()
        self.control_movement(distance_m, goal_m)

    def local_lock_motion(self, result: DetectionResult) -> None:
        self.center_camera()
        self.control_movement(result.distance_m, result.distance_m or 1.0, camera_only=True)

    def local_follow_motion(self, result: DetectionResult, desired_gap_m: float) -> None:
        self.center_camera()
        self.control_movement(result.distance_m, desired_gap_m)

    def send_camera(self, force=False):
        servo_x = clamp(CAMERA_CENTER_X + self.pan_angle, 50, 180)
        servo_y = clamp(self.camera_center_y + self.tilt_angle, 0, 180)
        now = time.time()
        same_position = self.last_camera_x == int(servo_x) and self.last_camera_y == int(servo_y)
        if not force and (same_position or now - self.last_camera_command < CAMERA_COMMAND_INTERVAL):
            return

        if self.control is not None:
            self.control.servo.set_servo_angle(0, int(servo_x))
            self.control.servo.set_servo_angle(1, int(servo_y))
        self.last_camera_command = now
        self.last_camera_x = int(servo_x)
        self.last_camera_y = int(servo_y)

    def send_move(self, y, angle=0):
        self.send_move_xy(0, y, angle)

    def send_move_xy(self, x, y, angle=0):
        if self.control is None:
            return
        x = clamp(int(x), -35, 35)
        y = clamp(int(y), -35, 35)
        angle = clamp(int(angle), -10, 10)
        self.control.command_queue = ["CMD_MOVE", "1", str(x), str(y), str(self.speed), str(angle)]
        self.control.timeout = time.time()

    def stand_up(self):
        if self.control is None:
            return
        self.control.servo_power_disable.off()
        time.sleep(0.2)
        self.control.command_queue = ["CMD_POSITION", "0", "0", str(self.body_z)]
        self.control.timeout = time.time()
        time.sleep(0.5)

    def request_ultrasonic_distance(self):
        now = time.time()
        if self.ultrasonic is not None and now - self.last_sonic_request >= SONIC_INTERVAL:
            self.obstacle_distance = self.ultrasonic.get_distance()
            self.last_sonic_request = now
        return self.obstacle_distance

    def center_camera(self):
        if abs(self.target_x) > CENTER_DEADZONE_X:
            pan_sign = 1 if self.invert_pan else -1
            self.pan_angle += pan_sign * signed_step(self.target_x, PAN_STEP_GAIN, MAX_PAN_STEP)
        if not self.lock_tilt and abs(self.target_y) > CENTER_DEADZONE_Y:
            tilt_sign = -1 if self.invert_tilt else 1
            self.tilt_angle += tilt_sign * signed_step(self.target_y, TILT_STEP_GAIN, MAX_TILT_STEP)

        self.pan_angle = clamp(self.pan_angle, -self.pan_max, self.pan_max)
        self.tilt_angle = clamp(self.tilt_angle, -self.tilt_max, self.tilt_max)
        self.send_camera()

    def turn_from_pan(self):
        if not self.enable_turning:
            return 0
        if abs(self.pan_angle) < TURN_DEADZONE_PAN and abs(self.target_x) < ROTATE_TARGET_X:
            return 0
        turn_source = self.pan_angle if abs(self.pan_angle) >= TURN_DEADZONE_PAN else -self.target_x * 20
        turn = clamp(-turn_source * TURN_GAIN, -5, 5)
        if abs(turn) < 1:
            turn = 1 if turn > 0 else -1
        return int(round(turn))

    def control_movement(self, distance_m, goal_m, camera_only=False):
        target_centered = (
            abs(self.target_x) <= CENTER_DEADZONE_X
            and (self.lock_tilt or abs(self.target_y) <= CENTER_DEADZONE_Y)
        )
        turn = self.turn_from_pan()

        if self.camera_only or camera_only:
            self.action_status = "camera-only"
            self.stop_motors()
        elif distance_m is not None and distance_m * 100.0 < OBSTACLE_MIN_CM:
            self.action_status = "obstacle-back"
            self.send_move(MOVE_BACKWARD_Y, 0)
        elif self.target_occluded:
            if turn != 0:
                self.action_status = "partial-target-rotate"
                self.send_move(0, turn)
            else:
                self.action_status = "partial-target-stop"
                self.stop_motors()
        elif turn != 0 and not target_centered:
            self.action_status = "rotate-to-center"
            self.send_move(0, turn)
        elif self.move_only_when_centered and not target_centered:
            self.action_status = "wait-center"
            self.stop_motors()
        elif self.target_area > self.target_area_max:
            self.action_status = "too-close-back"
            self.send_move(MOVE_BACKWARD_Y, 0)
        elif self.target_area < self.target_area_min:
            self.action_status = "forward"
            self.send_move(MOVE_FORWARD_Y, turn)
        elif turn != 0:
            self.action_status = "rotate"
            self.send_move(0, turn)
        else:
            self.action_status = "hold"
            self.stop_motors()

    def reset_camera(self):
        self.pan_angle = 0
        self.tilt_angle = 0
        self.send_camera(force=True)

    def close(self):
        self.stop_motors()
        self.reset_camera()
        if self.show_video:
            try:
                cv2.destroyWindow(self.video_window_name)
            except cv2.error:
                pass
        if self.camera is not None:
            try:
                self.camera.close()
            finally:
                self.camera = None
        if self.ultrasonic is not None:
            try:
                self.ultrasonic.close()
            finally:
                self.ultrasonic = None
