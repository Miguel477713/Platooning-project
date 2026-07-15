import os
import sys
import time
from pathlib import Path
from typing import Optional

import cv2
import numpy as np

from common.models import DetectionResult


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
MOVE_SPEED = 6
SPEED_MIN = 2
SPEED_MAX = 10
MOVE_FORWARD_Y = 12
MOVE_BACKWARD_Y = -10

PAN_MAX = 35
TILT_MAX = 35
CAMERA_CENTER_X = 90
CAMERA_CENTER_Y = 75
TURN_DEADZONE_PAN = 10
TURN_GAIN = 0.15
ROTATE_TARGET_X = 0.20
BODY_Z = 0
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

    def global_search_motion(self) -> None:
        print("[MOTOR] global search motion")

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

    This ports the usable pieces of Code/Client/FollowObjectProcedural.py into
    the state-machine implementation layer. It talks directly to the Freenove
    camera, ultrasonic sensor, servo controller, and gait Control class.
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

        detected, x, y, area, occluded = self.detect_target(frame, target_color)
        distance_cm = self.request_ultrasonic_distance()
        distance_m = None if distance_cm is None else distance_cm / 100.0

        if not detected:
            self.target_occluded = False
            return DetectionResult(detected=False, distance_m=distance_m)

        self.target_x = x
        self.target_y = y
        self.target_area = area
        self.target_occluded = occluded
        self.last_seen_time = time.time()

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
            return False, 0.0, 0.0, 0.0, False

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
            return False, 0.0, 0.0, 0.0, False

        contour = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(contour)
        height, width = frame.shape[:2]
        relative_area = area / float(width * height)
        if relative_area < 0.002:
            return False, 0.0, 0.0, 0.0, False

        moments = cv2.moments(contour)
        if moments["m00"] == 0:
            return False, 0.0, 0.0, 0.0, False

        center_x = int(moments["m10"] / moments["m00"])
        center_y = int(moments["m01"] / moments["m00"])
        target_x = (center_x - width / 2.0) / (width / 2.0)
        target_y = (center_y - height / 2.0) / (height / 2.0)

        x, y, w, h = cv2.boundingRect(contour)
        occluded, _ = self.detect_target_occlusion(frame, mask, x, y, w, h, relative_area)
        return True, target_x, target_y, relative_area, occluded

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

    def global_search_motion(self) -> None:
        self.action_status = "global-search-idle"
        self.stop_motors()

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
        if self.control is None:
            return
        y = clamp(int(y), -35, 35)
        angle = clamp(int(angle), -10, 10)
        self.control.command_queue = ["CMD_MOVE", "1", "0", str(y), str(self.speed), str(angle)]
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
        elif distance_m is not None and distance_m > goal_m:
            self.action_status = "forward"
            self.send_move(MOVE_FORWARD_Y, turn)
        elif distance_m is None and self.target_area < self.target_area_min:
            self.action_status = "forward-by-area"
            self.send_move(MOVE_FORWARD_Y, turn)
        elif self.target_area > self.target_area_max:
            self.action_status = "too-close-back"
            self.send_move(MOVE_BACKWARD_Y, 0)
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
