# -*- coding: utf-8 -*-
import argparse
import copy
import socket
import threading
import time

import cv2
import numpy as np

from Client import Client
from Command import COMMAND as cmd


CENTER_DEADZONE_X = 0.10
CENTER_DEADZONE_Y = 0.10

TARGET_AREA_MIN = 0.05
TARGET_AREA_MAX = 0.20

OBSTACLE_MIN_CM = 25.0
LOOP_DELAY = 0.05
SONIC_INTERVAL = 0.20
FRAME_TIMEOUT = 1.0
CAMERA_COMMAND_INTERVAL = 0.12

PAN_STEP_GAIN = 6
TILT_STEP_GAIN = 3
MAX_PAN_STEP = 4
MAX_TILT_STEP = 1
MOVE_SPEED = 6
SPEED_MIN = 2
SPEED_MAX = 10
MOVE_FORWARD_Y = 12
MOVE_BACKWARD_Y = -10

PAN_MIN = -35
PAN_MAX = 35
TILT_MIN = -35
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


class ProceduralObjectFollower:
    def __init__(
        self,
        client,
        color="red",
        speed=MOVE_SPEED,
        target_area=TARGET_AREA_MIN,
        pan_max=PAN_MAX,
        tilt_max=TILT_MAX,
        dry_run=False,
        camera_only=False,
        invert_pan=False,
        invert_tilt=False,
        lock_tilt=False,
        move_only_when_centered=True,
        wake_robot=True,
        enable_turning=True,
        camera_center_y=CAMERA_CENTER_Y,
        ignore_lower_frame=0.25,
        body_z=BODY_Z,
        occluded_target_area_max=OCCLUDED_TARGET_AREA_MAX,
        occlusion_dark_ratio=OCCLUSION_DARK_RATIO,
    ):
        self.client = client
        self.color = color
        self.speed = clamp(int(speed), SPEED_MIN, SPEED_MAX)
        self.target_area_min = clamp(target_area, 0.01, 0.90)
        self.target_area_max = self.target_area_min + 0.15
        self.dry_run = dry_run
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
        self.target_detected = False
        self.target_x = 0.0
        self.target_y = 0.0
        self.target_area = 0.0
        self.target_occluded = False
        self.action_status = "idle"
        self.last_seen_time = 0.0
        self.search_direction = 1
        self.last_sonic_request = 0.0
        self.obstacle_distance = None
        self.running = False
        self.receiver_thread = None
        self.last_camera_command = 0.0
        self.last_camera_x = None
        self.last_camera_y = None

    def start_response_receiver(self):
        if self.receiver_thread is not None:
            return
        self.receiver_thread = threading.Thread(target=self.receive_responses)
        self.receiver_thread.daemon = True
        self.receiver_thread.start()

    def receive_responses(self):
        while self.running:
            try:
                received = self.client.receive_data()
            except Exception as e:
                print(e)
                break
            if received == "":
                break
            for line in received.split("\n"):
                if line:
                    self.handle_response(line)

    def handle_response(self, line):
        data = line.split("#")
        if len(data) >= 2 and data[0] == cmd.CMD_SONIC:
            try:
                self.obstacle_distance = float(data[1])
            except ValueError:
                pass

    def detect_target(self, frame):
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = np.zeros(hsv.shape[:2], dtype=np.uint8)

        for lower, upper in COLOR_RANGES[self.color]:
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
            return False, 0.0, 0.0, 0.0, False, frame

        contour = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(contour)
        height, width = frame.shape[:2]
        relative_area = area / float(width * height)
        if relative_area < 0.002:
            return False, 0.0, 0.0, 0.0, False, frame

        moments = cv2.moments(contour)
        if moments["m00"] == 0:
            return False, 0.0, 0.0, 0.0, False, frame

        center_x = int(moments["m10"] / moments["m00"])
        center_y = int(moments["m01"] / moments["m00"])
        target_x = (center_x - width / 2.0) / (width / 2.0)
        target_y = (center_y - height / 2.0) / (height / 2.0)

        x, y, w, h = cv2.boundingRect(contour)
        annotated = frame.copy()
        occluded, occlusion_ratio = self.detect_target_occlusion(frame, mask, x, y, w, h, relative_area)
        cv2.rectangle(annotated, (x, y), (x + w, y + h), (0, 255, 0), 2)
        cv2.circle(annotated, (center_x, center_y), 4, (255, 0, 0), -1)
        cv2.line(annotated, (width // 2, 0), (width // 2, height), (255, 255, 255), 1)
        cv2.line(annotated, (0, height // 2), (width, height // 2), (255, 255, 255), 1)
        if occluded:
            cv2.putText(
                annotated,
                "PARTIAL TARGET dark={}".format(round(occlusion_ratio, 2)),
                (max(10, x), max(24, y - 8)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.50,
                (0, 0, 255),
                2,
                cv2.LINE_AA,
            )

        return True, target_x, target_y, relative_area, occluded, annotated

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

    def update_target_state(self, target_x, target_y, target_area, target_occluded):
        self.target_detected = True
        self.target_x = target_x
        self.target_y = target_y
        self.target_area = target_area
        self.target_occluded = target_occluded
        self.last_seen_time = time.time()

    def send_command(self, command):
        if self.dry_run:
            print(command.strip())
        else:
            self.client.send_data(command)

    def send_camera(self, force=False):
        servo_x = clamp(CAMERA_CENTER_X + self.pan_angle, 50, 180)
        servo_y = clamp(self.camera_center_y + self.tilt_angle, 0, 180)
        now = time.time()
        same_position = self.last_camera_x == int(servo_x) and self.last_camera_y == int(servo_y)
        if not force and (same_position or now - self.last_camera_command < CAMERA_COMMAND_INTERVAL):
            return

        command = cmd.CMD_CAMERA + "#" + str(int(servo_x)) + "#" + str(int(servo_y)) + "\n"
        self.send_command(command)
        self.last_camera_command = now
        self.last_camera_x = int(servo_x)
        self.last_camera_y = int(servo_y)

    def send_move(self, y, angle=0):
        y = clamp(int(y), -35, 35)
        angle = clamp(int(angle), -10, 10)
        command = (
            cmd.CMD_MOVE + "#1#0#" + str(y) + "#" + str(self.speed) + "#"
            + str(angle) + "\n"
        )
        self.send_command(command)

    def send_stop(self):
        self.send_move(0, 0)

    def stand_up(self):
        self.send_command(cmd.CMD_SERVOPOWER + "#1\n")
        time.sleep(0.2)
        self.send_command(cmd.CMD_POSITION + "#0#0#" + str(self.body_z) + "\n")
        time.sleep(0.5)

    def request_ultrasonic_distance(self):
        now = time.time()
        if now - self.last_sonic_request >= SONIC_INTERVAL:
            self.send_command(cmd.CMD_SONIC + "\n")
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

    def control_movement(self, obstacle_distance):
        target_centered = (
            abs(self.target_x) <= CENTER_DEADZONE_X
            and (self.lock_tilt or abs(self.target_y) <= CENTER_DEADZONE_Y)
        )

        turn = self.turn_from_pan()

        if self.camera_only:
            self.action_status = "camera-only"
            self.send_stop()
        elif obstacle_distance is not None and obstacle_distance < OBSTACLE_MIN_CM:
            self.action_status = "obstacle-back"
            self.send_move(MOVE_BACKWARD_Y, 0)
        elif self.target_occluded:
            if turn != 0:
                self.action_status = "partial-target-rotate"
                self.send_move(0, turn)
            else:
                self.action_status = "partial-target-stop"
                self.send_stop()
        elif turn != 0 and not target_centered:
            self.action_status = "rotate-to-center"
            self.send_move(0, turn)
        elif self.move_only_when_centered and not target_centered:
            self.action_status = "wait-center"
            self.send_stop()
        elif self.target_area < self.target_area_min:
            self.action_status = "forward"
            self.send_move(MOVE_FORWARD_Y, turn)
        elif self.target_area > self.target_area_max:
            self.action_status = "too-close-back"
            self.send_move(MOVE_BACKWARD_Y, 0)
        elif turn != 0:
            self.action_status = "rotate"
            self.send_move(0, turn)
        else:
            self.action_status = "hold"
            self.send_stop()

    def search_target(self):
        self.target_detected = False
        self.target_occluded = False
        self.action_status = "search"
        if time.time() - self.last_seen_time < 0.5:
            self.send_stop()
            return

        self.pan_angle += self.search_direction * 3
        if self.pan_angle > self.pan_max:
            self.pan_angle = self.pan_max
            self.search_direction = -1
        elif self.pan_angle < -self.pan_max:
            self.pan_angle = -self.pan_max
            self.search_direction = 1

        self.send_camera()
        self.send_stop()

    def annotate_status(self, frame):
        status = (
            "x={:.2f} y={:.2f} area={:.3f} pan={} tilt={} speed={} sonic={} {}".format(
                self.target_x,
                self.target_y,
                self.target_area,
                self.pan_angle,
                self.tilt_angle,
                self.speed,
                "--" if self.obstacle_distance is None else round(self.obstacle_distance, 1),
                self.action_status,
            )
        )
        cv2.putText(
            frame,
            status,
            (10, 24),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (0, 255, 255),
            2,
            cv2.LINE_AA,
        )
        if self.camera_only:
            cv2.putText(
                frame,
                "CAMERA ONLY",
                (10, 50),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                (0, 255, 255),
                2,
                cv2.LINE_AA,
            )
        if self.target_occluded:
            cv2.putText(
                frame,
                "PARTIAL/OCCLUDED TARGET: no distance move",
                (10, 50 if not self.camera_only else 76),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                (0, 0, 255),
                2,
                cv2.LINE_AA,
            )
        return frame

    def get_latest_frame(self):
        if self.client.video_flag:
            return None
        frame = copy.copy(self.client.image)
        self.client.video_flag = True
        return frame

    def reset_camera(self):
        self.pan_angle = 0
        self.tilt_angle = 0
        self.send_camera(force=True)

    def run(self, show_video=True):
        self.running = True
        self.start_response_receiver()
        if self.wake_robot:
            self.stand_up()
        self.reset_camera()
        self.send_stop()
        self.last_seen_time = time.time()
        last_frame_time = time.time()

        try:
            while self.running:
                frame = self.get_latest_frame()
                if frame is None:
                    if time.time() - last_frame_time > FRAME_TIMEOUT:
                        self.send_stop()
                    time.sleep(LOOP_DELAY)
                    continue

                last_frame_time = time.time()
                detected, x, y, area, occluded, annotated = self.detect_target(frame)
                obstacle_distance = self.request_ultrasonic_distance()

                if detected:
                    self.update_target_state(x, y, area, occluded)
                    self.center_camera()
                    self.control_movement(obstacle_distance)
                else:
                    self.target_occluded = False
                    self.search_target()

                if show_video:
                    annotated = self.annotate_status(annotated)
                    cv2.imshow("Procedural Object Follower", annotated)
                    if cv2.waitKey(1) & 0xFF == ord("q"):
                        break

                time.sleep(LOOP_DELAY)
        except KeyboardInterrupt:
            pass
        finally:
            self.running = False
            self.send_stop()
            self.reset_camera()
            if show_video:
                cv2.destroyAllWindows()


def connect_client(ip):
    client = Client()
    client.turn_on_client(ip)

    deadline = time.time() + 10.0
    last_error = None
    while time.time() < deadline:
        try:
            client.client_socket1.connect((ip, 5002))
            break
        except OSError as e:
            last_error = e
            print("Esperando servidor de comandos en " + ip + ":5002 ...")
            time.sleep(1.0)
    else:
        raise ConnectionError(
            "No se pudo conectar con " + ip + ":5002. "
            "Arranca el servidor en la Raspberry Pi desde Code/Server con "
            "`sudo python3 main.py -t -n`, o abre la UI del servidor y pulsa On. "
            "Si estas ejecutando este archivo por VNC en la misma Raspberry Pi, "
            "prueba tambien `--ip 127.0.0.1` solo si el servidor escucha en localhost. "
            "Error original: " + str(last_error)
        )

    client.tcp_flag = True

    video_thread = threading.Thread(target=client.receiving_video, args=(ip,))
    video_thread.daemon = True
    video_thread.start()
    return client


def parse_args():
    parser = argparse.ArgumentParser(
        description="Seguimiento procedural de un objeto de color para el hexapodo Freenove."
    )
    parser.add_argument("--ip", required=True, help="IP de la Raspberry Pi del robot.")
    parser.add_argument(
        "--color",
        choices=sorted(COLOR_RANGES.keys()),
        default="red",
        help="Color HSV del objetivo a seguir.",
    )
    parser.add_argument(
        "--speed",
        type=int,
        default=MOVE_SPEED,
        help="Velocidad de movimiento del robot (rango seguro de 2 a 10).",
    )
    parser.add_argument(
        "--target-area",
        type=float,
        default=TARGET_AREA_MIN,
        help="Area del objetivo para detenerse (mayor valor = se acerca mas. Ej: 0.15).",
    )
    parser.add_argument(
        "--pan-max",
        type=int,
        default=PAN_MAX,
        help="Angulo horizontal maximo de la camara (default 35, hasta 60).",
    )
    parser.add_argument(
        "--tilt-max",
        type=int,
        default=TILT_MAX,
        help="Angulo vertical maximo de la camara (default 35, hasta 60).",
    )
    parser.add_argument(
        "--no-video",
        action="store_true",
        help="Ejecuta el control sin abrir ventana de depuracion de OpenCV.",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Imprime comandos en vez de enviarlos por TCP.",
    )
    parser.add_argument(
        "--camera-only",
        action="store_true",
        help="Centra la camara pero mantiene las patas detenidas.",
    )
    parser.add_argument(
        "--invert-pan",
        action="store_true",
        help="Invierte la direccion horizontal de correccion de la camara.",
    )
    parser.add_argument(
        "--invert-tilt",
        action="store_true",
        help="Invierte la direccion vertical de correccion de la camara.",
    )
    parser.add_argument(
        "--lock-tilt",
        action="store_true",
        help="Bloquea el servo vertical y solo corrige pan horizontal.",
    )
    parser.add_argument(
        "--move-before-centered",
        action="store_true",
        help="Permite caminar aunque el objetivo aun no este centrado.",
    )
    parser.add_argument(
        "--enable-turning",
        action="store_true",
        help="Mantiene compatibilidad; la rotacion esta activa salvo que uses --disable-turning.",
    )
    parser.add_argument(
        "--disable-turning",
        action="store_true",
        help="Desactiva el giro del chasis para que el hexapodo solo camine en linea recta.",
    )
    parser.add_argument(
        "--camera-center-y",
        type=int,
        default=CAMERA_CENTER_Y,
        help="Angulo base vertical de la camara. Menor suele mirar mas alto.",
    )
    parser.add_argument(
        "--ignore-lower-frame",
        type=float,
        default=0.25,
        help="Fraccion inferior del video que se ignora para evitar detectar patas.",
    )
    parser.add_argument(
        "--body-z",
        type=int,
        default=BODY_Z,
        help="Altura corporal inicial enviada como CMD_POSITION#0#0#z. Rango del servidor: -20 a 20.",
    )
    parser.add_argument(
        "--occluded-target-area-max",
        type=float,
        default=OCCLUDED_TARGET_AREA_MAX,
        help="Area maxima para tratar una deteccion como parcial si hay una pata oscura alrededor.",
    )
    parser.add_argument(
        "--occlusion-dark-ratio",
        type=float,
        default=OCCLUSION_DARK_RATIO,
        help="Fraccion oscura alrededor del objetivo que marca una deteccion como parcialmente tapada.",
    )
    parser.add_argument(
        "--no-wake",
        action="store_true",
        help="No envia CMD_SERVOPOWER ni CMD_POSITION al iniciar.",
    )
    return parser.parse_args()


def main():
    args = parse_args()
    client = None
    try:
        if args.dry_run:
            client = Client()
        else:
            client = connect_client(args.ip)

        follower = ProceduralObjectFollower(
            client,
            color=args.color,
            speed=args.speed,
            target_area=args.target_area,
            pan_max=args.pan_max,
            tilt_max=args.tilt_max,
            dry_run=args.dry_run,
            camera_only=args.camera_only,
            invert_pan=args.invert_pan,
            invert_tilt=args.invert_tilt,
            lock_tilt=args.lock_tilt,
            move_only_when_centered=not args.move_before_centered,
            wake_robot=not args.no_wake,
            enable_turning=not args.disable_turning,
            camera_center_y=args.camera_center_y,
            ignore_lower_frame=args.ignore_lower_frame,
            body_z=args.body_z,
            occluded_target_area_max=args.occluded_target_area_max,
            occlusion_dark_ratio=args.occlusion_dark_ratio,
        )
        follower.run(show_video=not args.no_video)
    except (ConnectionError, ConnectionRefusedError, socket.timeout, OSError) as e:
        print("Error de conexion: " + str(e))
    finally:
        if client is not None and not args.dry_run:
            client.turn_off_client()


if __name__ == "__main__":
    main()
