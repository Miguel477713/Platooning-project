import time

from common.models import SuperintendentMeasurement
from common.parsing import parse_assignment
from common.topics import EMERGENCY_TOPIC


SUPERINTENDENT_ID = "Superintendent"


class FollowerMqttHandler:
    """MQTT message router for a follower robot.
    """

    def __init__(self, robot):
        self.robot = robot

    def handle_message(self, topic: str, data: dict) -> None:
        message_type = data.get("type")

        if topic == EMERGENCY_TOPIC:
            reason = data.get("reason", "unknown")
            self.robot.receive_emergency_stop(reason)
            return

        if topic.startswith("platoon/assignment/"):
            if message_type == "ASSIGNMENT":
                assignment = parse_assignment(data)

                if assignment is not None:
                    self.robot.receive_assignment(assignment)

            return

        if topic.startswith("platoon/command/"):
            self.handle_command_message(data)
            return

        if topic.startswith("platoon/robot/") and topic.endswith("/event"):
            self.handle_robot_event_message(data)
            return

    def handle_command_message(self, data: dict) -> None:
        message_type = data.get("type")

        if message_type == "STOP":
            self.robot.receive_stop()

        elif message_type == "RESET":
            self.robot.reset()

        elif message_type == "EMERGENCY_STOP":
            reason = data.get("reason", "command")
            self.robot.receive_emergency_stop(reason)

        elif message_type == "PING":
            self.robot.publish_event("PONG")

        else:
            print("[MQTT] unknown command type:", message_type)

    def handle_robot_event_message(self, data: dict) -> None:
        if self.robot.assignment is None:
            return

        source_robot_id = data.get("robot_id")
        event_name = data.get("event")

        if source_robot_id == self.robot.robot_id:
            return

        final_target_id = self.robot.assignment.final_target_id

        if (
            source_robot_id == SUPERINTENDENT_ID
            and event_name == "GLOBAL_SEARCH_MARKER_DISTANCE"
        ):
            self.handle_superintendent_distance(data)
            return

        target_is_ready = (
            source_robot_id == final_target_id
            and event_name in ["LOCAL_LOCK_ACQUIRED", "READY", "WAIT_ZONE_REACHED"]
        )

        if target_is_ready:
            self.robot.receive_final_target_ready(source_robot_id)

    def handle_superintendent_distance(self, data: dict) -> None:
        try:
            measurement = SuperintendentMeasurement(
                source_marker=str(data["source_marker"]),
                target_marker=str(data["target_marker"]),
                distance_m=self.optional_float(data.get("distance_m")),
                raw_distance_m=self.optional_float(data.get("raw_distance_m")),
                dx_m=self.optional_float(data.get("dx_m")),
                dy_m=self.optional_float(data.get("dy_m")),
                timestamp=time.time(),
                calibrated=bool(data.get("calibrated", False)),
                filtered=bool(data.get("filtered", False)),
            )
        except (KeyError, TypeError, ValueError) as error:
            print("[MQTT] invalid Superintendent distance event:", error)
            return

        self.robot.receive_superintendent_measurement(measurement)

    @staticmethod
    def optional_float(value):
        if value is None:
            return None
        return float(value)
