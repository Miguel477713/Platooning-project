import threading
from typing import Optional

from common.topics import robot_event_topic, robot_status_topic
from mqtt.transport import PahoMqttTransport


READY_EVENTS = {"READY", "LOCAL_LOCK_ACQUIRED", "WAIT_ZONE_REACHED"}
READY_STATES = set()
READY_LOCAL_FOLLOW_ACTION_STATUSES = {"hold"}


class FollowerReadySignal:
    """MQTT-backed gate used by the leader to wait for a follower readiness signal."""

    def __init__(self, broker: str, port: int, follower_id: str):
        self.broker = broker
        self.port = port
        self.follower_id = follower_id
        self.ready = threading.Event()
        self.last_signal: Optional[dict] = None

        self.transport = PahoMqttTransport(
            client_id="LineaPausa-leader",
            subscriptions=[
                (robot_event_topic(follower_id), 1),
                (robot_status_topic(follower_id), 0),
            ],
            on_message=self.handle_message,
        )

    def start(self) -> None:
        self.transport.connect(self.broker, self.port, keepalive=30)
        self.transport.loop_start()

    def stop(self) -> None:
        self.transport.loop_stop()
        self.transport.disconnect()

    def clear(self) -> None:
        self.last_signal = None
        self.ready.clear()

    def consume(self) -> Optional[dict]:
        if not self.ready.is_set():
            return None

        signal = self.last_signal
        self.clear()
        return signal

    def handle_message(self, topic: str, data: dict) -> None:
        if data.get("robot_id") != self.follower_id:
            return

        message_type = data.get("type")
        if message_type == "EVENT" and data.get("event") in READY_EVENTS:
            self.last_signal = data
            self.ready.set()
            print("[MQTT] seguidor listo:", data.get("event"))
        elif message_type == "STATUS" and data.get("state") in READY_STATES:
            self.last_signal = data
            self.ready.set()
            print("[MQTT] seguidor listo por estado:", data.get("state"))
        elif (
            message_type == "STATUS"
            and data.get("state") == "LOCAL_FOLLOW"
            and data.get("action_status") in READY_LOCAL_FOLLOW_ACTION_STATUSES
        ):
            self.last_signal = data
            self.ready.set()
            print(
                "[MQTT] seguidor listo por espera:",
                data.get("state"),
                data.get("action_status"),
            )
