from common.topics import robot_event_topic, robot_status_topic


class FollowerMqttPublisher:
    """Publishes follower status/events through the generic MQTT transport."""

    def __init__(self, robot_id: str, transport):
        self.robot_id = robot_id
        self.transport = transport

    def publish_status(self, status: dict) -> None:
        self.transport.publish(
            robot_status_topic(self.robot_id),
            payload=status,
            qos=0,
            retain=False,
        )

    def publish_event(self, event: dict) -> None:
        self.transport.publish(
            robot_event_topic(self.robot_id),
            payload=event,
            qos=1,
            retain=False,
        )
