import json
from typing import Callable, Iterable, Tuple, Optional

import paho.mqtt.client as mqtt


Subscription = Tuple[str, int]


class PahoMqttTransport:
    """Generic MQTT transport adapter.
    """

    def __init__(
        self,
        client_id: str,
        subscriptions: Iterable[Subscription],
        on_message: Callable[[str, dict], None],
        on_connected: Optional[Callable[[], None]] = None,
    ):
        self.client_id = client_id
        self.subscriptions = list(subscriptions)
        self.on_message_callback = on_message
        self.on_connected_callback = on_connected
        self.client = self._make_client(client_id)

        self.client.on_connect = self._on_connect
        self.client.on_disconnect = self._on_disconnect
        self.client.on_message = self._on_message

    @staticmethod
    def _make_client(client_id: str):
        try:
            return mqtt.Client(
                mqtt.CallbackAPIVersion.VERSION2,
                client_id=client_id,
            )
        except AttributeError:
            return mqtt.Client(client_id=client_id)

    def connect(self, broker: str, port: int, keepalive: int = 30) -> None:
        self.client.connect(broker, port, keepalive=keepalive)

    def loop_start(self) -> None:
        self.client.loop_start()

    def loop_forever(self) -> None:
        self.client.loop_forever()

    def loop_stop(self) -> None:
        self.client.loop_stop()

    def disconnect(self) -> None:
        self.client.disconnect()

    def publish(self, topic: str, payload: Optional[dict], qos: int = 0, retain: bool = False) -> None:
        if payload is None:
            encoded_payload = None
        else:
            encoded_payload = json.dumps(payload)

        self.client.publish(
            topic,
            payload=encoded_payload,
            qos=qos,
            retain=retain,
        )

    def _on_connect(self, client, userdata, flags, reason_code, properties=None) -> None:
        print("[MQTT] connected:", reason_code)

        for topic, qos in self.subscriptions:
            client.subscribe(topic, qos=qos)
            print(f"[MQTT] subscribed topic={topic} qos={qos}")

        if self.on_connected_callback is not None:
            self.on_connected_callback()

    def _on_disconnect(self, client, userdata, disconnect_flags=None, reason_code=None, properties=None) -> None:
        print("[MQTT] disconnected")

    def _on_message(self, client, userdata, msg) -> None:
        try:
            payload_text = msg.payload.decode("utf-8")
            data = json.loads(payload_text)
        except ValueError:
            print("[MQTT] invalid JSON:", msg.topic, msg.payload)
            return

        print("[MQTT] message:", msg.topic, data)
        self.on_message_callback(msg.topic, data)
