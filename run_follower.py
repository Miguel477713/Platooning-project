import argparse
import time

from common.models import Assignment
from common.topics import (
    ASSIGNMENT_ALL_TOPIC,
    COMMAND_TOPIC_FILTER,
    EMERGENCY_TOPIC,
    ROBOT_EVENT_TOPIC_FILTER,
    assignment_topic,
)
from follower.robot_implementation import (
    BODY_Z,
    CAMERA_CENTER_Y,
    MOVE_SPEED,
    OCCLUDED_TARGET_AREA_MAX,
    OCCLUSION_DARK_RATIO,
    PAN_MAX,
    TARGET_AREA_MIN,
    TILT_MAX,
    COLOR_RANGES,
    FreenoveDirectRobotImplementation,
    FollowerRobotImplementation,
)
from follower.state_machine import FollowerStateMachine
from mqtt.follower_handler import FollowerMqttHandler
from mqtt.follower_publisher import FollowerMqttPublisher
from mqtt.transport import PahoMqttTransport


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--robot-id", required=True, help="Example: Hexapod1 or Hexapod2")
    parser.add_argument("--broker", default="10.0.7.51", help="MQTT broker IP address")
    parser.add_argument("--port", type=int, default=1883, help="MQTT broker port")
    parser.add_argument("--period", type=float, default=0.1, help="Control loop period in seconds")
    parser.add_argument(
        "--no-mqtt",
        action="store_true",
        help="Run only the local state machine; useful for direct single-target approach tests.",
    )
    parser.add_argument(
        "--target-color",
        choices=sorted(COLOR_RANGES.keys()),
        help="Start with a local assignment for this color instead of waiting for MQTT.",
    )
    parser.add_argument(
        "--target-id",
        default="manual-target",
        help="Target id used with --target-color.",
    )
    parser.add_argument(
        "--approach-distance",
        type=float,
        default=0.35,
        help="Goal distance in meters used with --target-color.",
    )
    parser.add_argument(
        "--mock",
        action="store_true",
        help="Use placeholder print-only robot implementation instead of Freenove hardware.",
    )
    parser.add_argument("--speed", type=int, default=MOVE_SPEED, help="Freenove gait speed, 2 to 10.")
    parser.add_argument(
        "--target-area",
        type=float,
        default=TARGET_AREA_MIN,
        help="Visual target area used as approach fallback when ultrasonic distance is unavailable.",
    )
    parser.add_argument("--pan-max", type=int, default=PAN_MAX, help="Maximum camera pan offset.")
    parser.add_argument("--tilt-max", type=int, default=TILT_MAX, help="Maximum camera tilt offset.")
    parser.add_argument("--camera-only", action="store_true", help="Track with camera but do not walk.")
    parser.add_argument("--show-video", action="store_true", help="Show direct camera debug window.")
    parser.add_argument("--invert-pan", action="store_true", help="Invert pan correction.")
    parser.add_argument("--invert-tilt", action="store_true", help="Invert tilt correction.")
    parser.add_argument("--lock-tilt", action="store_true", help="Keep vertical camera servo fixed.")
    parser.add_argument(
        "--move-before-centered",
        action="store_true",
        help="Allow walking before the target is centered.",
    )
    parser.add_argument(
        "--disable-turning",
        action="store_true",
        help="Disable chassis rotation while approaching.",
    )
    parser.add_argument(
        "--camera-center-y",
        type=int,
        default=CAMERA_CENTER_Y,
        help="Base vertical camera angle.",
    )
    parser.add_argument(
        "--ignore-lower-frame",
        type=float,
        default=0.25,
        help="Lower image fraction ignored by color detection.",
    )
    parser.add_argument("--body-z", type=int, default=BODY_Z, help="Initial body height command.")
    parser.add_argument(
        "--occluded-target-area-max",
        type=float,
        default=OCCLUDED_TARGET_AREA_MAX,
        help="Maximum visual area for partial-target occlusion checks.",
    )
    parser.add_argument(
        "--occlusion-dark-ratio",
        type=float,
        default=OCCLUSION_DARK_RATIO,
        help="Dark-pixel ratio that marks a small detection as occluded.",
    )
    parser.add_argument("--no-wake", action="store_true", help="Do not stand up on startup.")
    parser.add_argument(
        "--superintendent-source-marker",
        help="Overhead marker attached to this robot, for example green.",
    )
    parser.add_argument(
        "--superintendent-target-marker",
        help="Overhead target marker this robot should move toward during GLOBAL_SEARCH.",
    )
    args = parser.parse_args()

    if args.no_mqtt and args.target_color is None:
        parser.error("--no-mqtt requires --target-color so the state machine has an assignment.")
    superintendent_args = [
        args.superintendent_source_marker,
        args.superintendent_target_marker,
    ]
    if any(superintendent_args) and not all(superintendent_args):
        parser.error(
            "--superintendent-source-marker and --superintendent-target-marker "
            "must be used together."
        )

    if args.mock:
        implementation = FollowerRobotImplementation()
    else:
        implementation = FreenoveDirectRobotImplementation(
            speed=args.speed,
            target_area=args.target_area,
            pan_max=args.pan_max,
            tilt_max=args.tilt_max,
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
            show_video=args.show_video,
        )
    robot = FollowerStateMachine(
        robot_id=args.robot_id,
        implementation=implementation,
        superintendent_source_marker=args.superintendent_source_marker,
        superintendent_target_marker=args.superintendent_target_marker,
    )
    handler = FollowerMqttHandler(robot)

    transport = None
    if not args.no_mqtt:
        subscriptions = [
            (assignment_topic(args.robot_id), 1),
            (ASSIGNMENT_ALL_TOPIC, 1),
            (COMMAND_TOPIC_FILTER, 1),
            (EMERGENCY_TOPIC, 1),
            (ROBOT_EVENT_TOPIC_FILTER, 1),
        ]

        transport = PahoMqttTransport(
            client_id=args.robot_id,
            subscriptions=subscriptions,
            on_message=handler.handle_message,
            on_connected=lambda: robot.publish_event("ONLINE"),
        )

        publisher = FollowerMqttPublisher(robot_id=args.robot_id, transport=transport)
        robot.set_publisher(publisher)

        transport.connect(args.broker, args.port, keepalive=30)
        transport.loop_start()

    if args.target_color is not None:
        robot.receive_assignment(
            Assignment(
                robot_id=args.robot_id,
                initial_target_id=args.target_id,
                initial_target_color=args.target_color,
                final_target_id=args.target_id,
                final_target_color=args.target_color,
                initial_wait_distance_m=args.approach_distance,
                desired_gap_m=args.approach_distance,
            )
        )

    print("[INFO] follower started")
    print("[INFO] robot_id:", args.robot_id)
    if args.no_mqtt:
        print("[INFO] mqtt: disabled")
    else:
        print("[INFO] broker:", args.broker)
    if args.target_color is None:
        print("[INFO] waiting for assignment...")
    else:
        print("[INFO] local assignment:", args.target_id, args.target_color)
    if args.show_video:
        print("[INFO] video: showing direct camera debug window")
    if args.superintendent_source_marker is not None:
        print(
            "[INFO] superintendent guidance:",
            args.superintendent_source_marker,
            "to",
            args.superintendent_target_marker,
        )

    try:
        while True:
            robot.step()
            time.sleep(args.period)

    except KeyboardInterrupt:
        print("[INFO] stopping")
        implementation.stop_motors()
        if hasattr(implementation, "close"):
            implementation.close()
        robot.publish_event("OFFLINE")
        if transport is not None:
            transport.loop_stop()
            transport.disconnect()


if __name__ == "__main__":
    main()
