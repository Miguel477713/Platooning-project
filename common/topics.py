def assignment_topic(robot_id: str) -> str:
    return f"platoon/assignment/{robot_id}"


ASSIGNMENT_ALL_TOPIC = "platoon/assignment/all"
COMMAND_TOPIC_FILTER = "platoon/command/#"
EMERGENCY_TOPIC = "platoon/emergency"
ROBOT_EVENT_TOPIC_FILTER = "platoon/robot/+/event"


def robot_status_topic(robot_id: str) -> str:
    return f"platoon/robot/{robot_id}/status"


def robot_event_topic(robot_id: str) -> str:
    return f"platoon/robot/{robot_id}/event"
