import argparse
import time

from common.topics import (
    ASSIGNMENT_ALL_TOPIC,
    COMMAND_TOPIC_FILTER,
    EMERGENCY_TOPIC,
    ROBOT_EVENT_TOPIC_FILTER,
    assignment_topic,
)
from follower.robot_implementation import FollowerRobotImplementation
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
    args = parser.parse_args()

    implementation = FollowerRobotImplementation()
    robot = FollowerStateMachine(robot_id=args.robot_id, implementation=implementation)
    handler = FollowerMqttHandler(robot)

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

    print("[INFO] follower started")
    print("[INFO] robot_id:", args.robot_id)
    print("[INFO] broker:", args.broker)
    print("[INFO] waiting for assignment...")

    try:
        while True:
            robot.step()
            time.sleep(args.period)

    except KeyboardInterrupt:
        print("[INFO] stopping")
        implementation.stop_motors()
        robot.publish_event("OFFLINE")
        transport.loop_stop()
        transport.disconnect()


if __name__ == "__main__":
    main()
