import time
from typing import Optional, Dict, Any

from common.models import Assignment, DetectionResult, State
from follower.publisher import NullFollowerPublisher
from follower.robot_implementation import FollowerRobotImplementation


class FollowerStateMachine:
    """Pure follower state-machine logic.
    """

    def __init__(
        self,
        robot_id: str,
        implementation: Optional[FollowerRobotImplementation] = None,
        publisher=None,
    ):
        self.robot_id = robot_id
        self.state = State.WAIT_FOR_ASSIGNMENT

        self.assignment: Optional[Assignment] = None
        self.current_target_id: Optional[str] = None
        self.current_target_color: Optional[str] = None

        self.final_target_ready = False
        self.last_seen_time = 0.0
        self.local_lock_counter = 0
        self.target_lost_timeout_s = 1.0
        self.required_local_lock_frames = 10
        self.last_status_time = 0.0

        self.impl = implementation or FollowerRobotImplementation()
        self.publisher = publisher or NullFollowerPublisher()

    def set_publisher(self, publisher) -> None:
        self.publisher = publisher

    # =====================================================
    # Inputs from communication layer
    # =====================================================

    def receive_assignment(self, assignment: Assignment) -> None:
        if assignment.robot_id != self.robot_id:
            return

        self.assignment = assignment
        self.current_target_id = assignment.initial_target_id
        self.current_target_color = assignment.initial_target_color
        self.final_target_ready = False
        self.local_lock_counter = 0

        self.publish_event(
            "ASSIGNMENT_RECEIVED",
            {
                "initial_target_id": assignment.initial_target_id,
                "final_target_id": assignment.final_target_id,
            },
        )

        self.transition_to(State.GLOBAL_SEARCH)

    def receive_final_target_ready(self, target_id: str) -> None:
        if self.assignment is None:
            return

        if target_id == self.assignment.final_target_id:
            self.final_target_ready = True
            self.publish_event("FINAL_TARGET_READY_RECEIVED", {"target_id": target_id})

    def receive_emergency_stop(self, reason: str = "unknown") -> None:
        self.publish_event("EMERGENCY_STOP_RECEIVED", {"reason": reason})
        self.transition_to(State.EMERGENCY_STOP)

    def receive_stop(self) -> None:
        self.publish_event("STOP_RECEIVED")
        self.transition_to(State.EMERGENCY_STOP)

    def reset(self) -> None:
        self.impl.stop_motors()
        self.assignment = None
        self.current_target_id = None
        self.current_target_color = None
        self.final_target_ready = False
        self.local_lock_counter = 0
        self.transition_to(State.WAIT_FOR_ASSIGNMENT)

    # =====================================================
    # Main loop
    # =====================================================

    def step(self) -> None:
        now = time.time()

        if now - self.last_status_time >= 1.0:
            self.publish_status()
            self.last_status_time = now

        if self.state == State.WAIT_FOR_ASSIGNMENT:
            self.handle_wait_for_assignment()
        elif self.state == State.GLOBAL_SEARCH:
            self.handle_global_search()
        elif self.state == State.GLOBAL_APPROACH:
            self.handle_global_approach()
        elif self.state == State.WAIT_FOR_FINAL_TARGET_READY:
            self.handle_wait_for_final_target_ready()
        elif self.state == State.LOCAL_LOCK:
            self.handle_local_lock()
        elif self.state == State.LOCAL_FOLLOW:
            self.handle_local_follow()
        elif self.state == State.LOST_TARGET:
            self.handle_lost_target()
        elif self.state == State.EMERGENCY_STOP:
            self.handle_emergency_stop()

    # =====================================================
    # State handlers
    # =====================================================

    def handle_wait_for_assignment(self) -> None:
        self.impl.stop_motors()

    def handle_global_search(self) -> None:
        result = self.impl.global_detect(self.current_target_color)

        if result.detected:
            self.last_seen_time = time.time()
            self.publish_event(
                "TARGET_FOUND_GLOBAL",
                {
                    "target_id": self.current_target_id,
                    "target_color": self.current_target_color,
                    "distance_m": result.distance_m,
                    "bearing_deg": result.bearing_deg,
                    "confidence": result.confidence,
                },
            )
            self.transition_to(State.GLOBAL_APPROACH)
        else:
            self.impl.global_search_motion()

    def handle_global_approach(self) -> None:
        result = self.impl.global_detect(self.current_target_color)

        if not result.detected:
            self.impl.stop_motors()

            if self.target_has_been_lost_too_long():
                self.publish_event("TARGET_LOST_GLOBAL", {"target_id": self.current_target_id})
                self.transition_to(State.LOST_TARGET)

            return

        self.last_seen_time = time.time()

        if self.reached_global_approach_goal(result):
            self.impl.stop_motors()
            self.publish_event(
                "GLOBAL_APPROACH_COMPLETE",
                {
                    "target_id": self.current_target_id,
                    "distance_m": result.distance_m,
                },
            )

            if self.assignment and self.current_target_id == self.assignment.final_target_id:
                self.transition_to(State.LOCAL_LOCK)
            else:
                self.transition_to(State.WAIT_FOR_FINAL_TARGET_READY)

            return

        self.impl.global_approach_motion(
            target_id=self.current_target_id,
            distance_m=result.distance_m,
            goal_m=self.get_current_approach_distance(),
        )

    def handle_wait_for_final_target_ready(self) -> None:
        self.impl.stop_motors()

        if self.final_target_ready and self.assignment is not None:
            self.current_target_id = self.assignment.final_target_id
            self.current_target_color = self.assignment.final_target_color

            self.publish_event(
                "SWITCHING_TO_FINAL_TARGET",
                {
                    "final_target_id": self.current_target_id,
                    "final_target_color": self.current_target_color,
                },
            )

            self.transition_to(State.GLOBAL_SEARCH)

    def handle_local_lock(self) -> None:
        result = self.impl.local_detect(self.current_target_color)

        if not result.detected:
            self.impl.stop_motors()
            self.publish_event("LOCAL_LOCK_FAILED", {"target_id": self.current_target_id})
            self.transition_to(State.LOST_TARGET)
            return

        self.impl.local_lock_motion(result)

        if self.local_lock_is_stable(result):
            self.local_lock_counter += 1
        else:
            self.local_lock_counter = 0

        if self.local_lock_counter >= self.required_local_lock_frames:
            self.publish_event(
                "LOCAL_LOCK_ACQUIRED",
                {
                    "target_id": self.current_target_id,
                    "distance_m": result.distance_m,
                    "confidence": result.confidence,
                },
            )
            self.transition_to(State.LOCAL_FOLLOW)

    def handle_local_follow(self) -> None:
        result = self.impl.local_detect(self.current_target_color)

        if not result.detected:
            self.impl.stop_motors()
            self.publish_event("TARGET_LOST_LOCAL", {"target_id": self.current_target_id})
            self.transition_to(State.LOST_TARGET)
            return

        desired_gap_m = 1.0
        if self.assignment is not None:
            desired_gap_m = self.assignment.desired_gap_m

        self.impl.local_follow_motion(result, desired_gap_m=desired_gap_m)

    def handle_lost_target(self) -> None:
        self.impl.stop_motors()
        self.transition_to(State.GLOBAL_SEARCH)

    def handle_emergency_stop(self) -> None:
        self.impl.stop_motors()

    # =====================================================
    # Decision helpers
    # =====================================================

    def reached_global_approach_goal(self, result: DetectionResult) -> bool:
        if self.assignment is None:
            return False
        if result.distance_m is None:
            return False

        target_distance = self.get_current_approach_distance()
        return result.distance_m <= target_distance

    def get_current_approach_distance(self) -> float:
        if self.assignment is None:
            return 1.0

        is_initial_only_target = (
            self.current_target_id == self.assignment.initial_target_id
            and self.current_target_id != self.assignment.final_target_id
        )

        if is_initial_only_target:
            return self.assignment.initial_wait_distance_m

        return self.assignment.desired_gap_m

    def local_lock_is_stable(self, result: DetectionResult) -> bool:
        if not result.detected:
            return False
        if result.distance_m is None:
            return False
        if result.confidence < 0.7:
            return False
        return True

    def target_has_been_lost_too_long(self) -> bool:
        elapsed = time.time() - self.last_seen_time
        return elapsed > self.target_lost_timeout_s

    # =====================================================
    # Output/publishing helpers
    # =====================================================

    def transition_to(self, new_state: State) -> None:
        old_state = self.state
        self.state = new_state
        print(f"[STATE] {old_state.name} -> {new_state.name}")
        self.publish_status()

    def make_status_payload(self) -> Dict[str, Any]:
        return {
            "type": "STATUS",
            "robot_id": self.robot_id,
            "state": self.state.name,
            "current_target_id": self.current_target_id,
            "current_target_color": self.current_target_color,
            "timestamp": time.time(),
        }

    def make_event_payload(self, event_name: str, extra: Optional[dict] = None) -> Dict[str, Any]:
        payload = {
            "type": "EVENT",
            "robot_id": self.robot_id,
            "event": event_name,
            "state": self.state.name,
            "timestamp": time.time(),
        }

        if extra:
            payload.update(extra)

        return payload

    def publish_status(self) -> None:
        self.publisher.publish_status(self.make_status_payload())

    def publish_event(self, event_name: str, extra: Optional[dict] = None) -> None:
        self.publisher.publish_event(self.make_event_payload(event_name, extra))
