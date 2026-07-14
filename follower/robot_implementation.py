from common.models import DetectionResult


class FollowerRobotImplementation:
    """Hardware/vision implementation layer.
    """

    # =====================================================
    # Vision placeholders
    # =====================================================

    def global_detect(self, target_color: str) -> DetectionResult:
        """Full-frame/global blob detection.

        Replace with real camera/blob detection code.
        """
        print(f"[VISION] global search for color={target_color}")
        return DetectionResult(detected=False)

    def local_detect(self, target_color: str) -> DetectionResult:
        """Local/ROI blob tracking.

        Replace with real camera/blob tracking code.
        """
        print(f"[VISION] local search for color={target_color}")
        return DetectionResult(detected=False)

    # =====================================================
    # Motor placeholders
    # =====================================================

    def stop_motors(self) -> None:
        print("[MOTOR] stop")

    def global_search_motion(self) -> None:
        print("[MOTOR] global search motion")

    def global_approach_motion(self, *, target_id: str, distance_m: float, goal_m: float) -> None:
        print(
            "[MOTOR] global approach motion "
            f"target={target_id}, distance={distance_m}, goal={goal_m}"
        )

    def local_lock_motion(self, result: DetectionResult) -> None:
        print("[MOTOR] local lock alignment")

    def local_follow_motion(self, result: DetectionResult, desired_gap_m: float) -> None:
        error_m = None
        if result.distance_m is not None:
            error_m = result.distance_m - desired_gap_m

        print(
            "[MOTOR] local follow motion "
            f"distance={result.distance_m}, desired_gap={desired_gap_m}, error={error_m}"
        )
