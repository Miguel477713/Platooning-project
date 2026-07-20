import time
import threading
from collections.abc import Iterable
from typing import Callable, Dict, Any, Union

from common.topics import robot_event_topic
from mqtt.transport import PahoMqttTransport


class DistanceMqttPublisher:
    """Publishes distance payloads through the shared MQTT transport."""

    def __init__(
        self,
        *,
        broker: str,
        port: int,
        robot_id: str,
        period: float,
        payload_provider: Callable[[], Union[Dict[str, Any], Iterable[Dict[str, Any]]]],
    ):
        self.broker = broker
        self.port = port
        self.robot_id = robot_id
        self.period = period
        self.payload_provider = payload_provider
        self.topic = robot_event_topic(robot_id)
        self.running = False
        self.thread = None
        self.transport = PahoMqttTransport(
            client_id=f"distance-detection-{robot_id}",
            subscriptions=[],
            on_message=self.HandleMessage,
        )

    def HandleMessage(self, topic, data):
        return

    def Start(self):
        if self.thread and self.thread.is_alive():
            return

        self.transport.connect(self.broker, self.port, keepalive=30)
        self.transport.loop_start()
        self.running = True
        self.thread = threading.Thread(target=self.Loop, daemon=True)
        self.thread.start()

    def Loop(self):
        delay = max(self.period, 0.05)

        while self.running:
            payloads = self.payload_provider()
            if isinstance(payloads, dict):
                payloads = [payloads]

            for payload in payloads:
                self.transport.publish(
                    self.topic,
                    payload=payload,
                    qos=1,
                    retain=False,
                )
            time.sleep(delay)

    def Stop(self):
        self.running = False
        if self.thread is not None:
            self.thread.join(timeout=2.0)
        self.transport.loop_stop()
        self.transport.disconnect()
