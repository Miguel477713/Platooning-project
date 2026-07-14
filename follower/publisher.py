from typing import Optional, Dict, Any


class NullFollowerPublisher:
    """Publisher used when MQTT is not connected, for offline tests."""

    def publish_status(self, status: Dict[str, Any]) -> None:
        pass

    def publish_event(self, event: Dict[str, Any]) -> None:
        pass


class ConsoleFollowerPublisher:
    """Simple publisher for unit/simulation tests without MQTT."""

    def publish_status(self, status: Dict[str, Any]) -> None:
        print("[STATUS]", status)

    def publish_event(self, event: Dict[str, Any]) -> None:
        print("[EVENT]", event)
