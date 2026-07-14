from typing import Optional

from common.models import Assignment


def parse_assignment(data: dict) -> Optional[Assignment]:
    try:
        return Assignment(
            robot_id=data["robot_id"],
            initial_target_id=data["initial_target_id"],
            initial_target_color=data["initial_target_color"],
            final_target_id=data["final_target_id"],
            final_target_color=data["final_target_color"],
            initial_wait_distance_m=float(data["initial_wait_distance_m"]),
            desired_gap_m=float(data["desired_gap_m"]),
        )
    except KeyError as error:
        print("[PARSER] assignment missing field:", error)
        return None
    except ValueError as error:
        print("[PARSER] assignment value error:", error)
        return None
