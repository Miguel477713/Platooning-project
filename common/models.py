from dataclasses import dataclass
from enum import Enum, auto
from typing import Optional, Dict, Any


class State(Enum):
    WAIT_FOR_ASSIGNMENT = auto()
    GLOBAL_SEARCH = auto()
    GLOBAL_APPROACH = auto()
    WAIT_FOR_FINAL_TARGET_READY = auto()
    LOCAL_LOCK = auto()
    LOCAL_FOLLOW = auto()
    LOST_TARGET = auto()
    EMERGENCY_STOP = auto()


@dataclass
class Assignment:
    robot_id: str

    # First target used to find the group.
    # Example: Hexapod1 initial target = Cart.
    initial_target_id: str
    initial_target_color: str

    # Final target used for platooning.
    # Example: Hexapod2 final target = Hexapod1.
    final_target_id: str
    final_target_color: str

    # Distance used during the first approach.
    initial_wait_distance_m: float

    # Final platooning distance.
    desired_gap_m: float


@dataclass
class DetectionResult:
    detected: bool
    distance_m: Optional[float] = None
    bearing_deg: Optional[float] = None
    confidence: float = 0.0
    target_x: Optional[float] = None
    target_y: Optional[float] = None
    target_area: Optional[float] = None
    occluded: bool = False


@dataclass
class OutgoingMqttMessage:
    topic: str
    payload: Optional[Dict[str, Any]]
    qos: int = 0
    retain: bool = False
