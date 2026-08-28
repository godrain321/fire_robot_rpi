"""Stage 8 acceptance: mission waypoints provide goals, never path points."""

from pathlib import Path as FilePath
from types import SimpleNamespace

from builtin_interfaces.msg import Time
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String

from inno_autonav.waypoint_queue import WaypointQueue


SOURCE = FilePath(__file__).parents[1] / "inno_autonav" / "waypoint_queue.py"


class Publisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


def waypoint(x, y):
    message = PoseStamped()
    message.header.frame_id = "map"
    message.pose.position.x = float(x)
    message.pose.position.y = float(y)
    message.pose.orientation.w = 1.0
    return message


def queue_node(points):
    value = object.__new__(WaypointQueue)
    value.queue = list(points)
    value.waypoint_names = [f"exit{index + 1}" for index in range(len(points))]
    value.current_index = None
    value.waiting_for_departure = False
    value.selected_names = []
    value.selected_indices = []
    value.selected_next_position = 0
    value.selected_current_position = None
    value.step_index = 0
    value.execution_mode = "continuous"
    value.goal = Publisher()
    value.autonomy_cancel = Publisher()
    value.get_clock = lambda: SimpleNamespace(
        now=lambda: SimpleNamespace(to_msg=lambda: Time())
    )
    value._publish_queue = lambda: None
    value._state = lambda _state: None
    return value


def test_1_single_waypoint_publishes_only_the_existing_planner_goal():
    queue = queue_node([waypoint(4.0, 1.0)])
    queue._command(String(data="GO"))

    assert queue.current_index == 0
    assert queue.goal.messages == [queue.queue[0]]
    source = SOURCE.read_text(encoding="utf-8")
    assert "self.goal = self.create_publisher(PoseStamped, '/goal_pose'" in source
    assert "'/planned_path'" not in source


def test_2_second_waypoint_activates_once_only_after_first_goal_reached():
    queue = queue_node([waypoint(1.0, 0.0), waypoint(2.0, 0.0)])
    queue._command(String(data="GO"))
    queue._follower(String(data="FOLLOWING_PATH"))
    assert len(queue.goal.messages) == 1

    queue._follower(String(data="GOAL_REACHED"))
    assert queue.current_index == 1
    assert queue.goal.messages == [queue.queue[0], queue.queue[1]]


def test_3_replanned_paths_keep_current_mission_goal_and_queue_unchanged():
    points = [waypoint(4.0, 1.0), waypoint(8.0, 1.0)]
    queue = queue_node(points)
    queue._command(String(data="GO"))
    original_queue = tuple(queue.queue)

    # Thermal/dynamic replans are observed as newly accepted/followed paths.
    for state in ("PATH_ACCEPTED", "FOLLOWING_PATH") * 4:
        queue._follower(String(data=state))

    assert queue.current_index == 0
    assert tuple(queue.queue) == original_queue
    assert queue.goal.messages == [queue.queue[0]]


def test_5_many_path_updates_never_complete_a_waypoint():
    queue = queue_node([waypoint(1.0, 0.0), waypoint(2.0, 0.0)])
    queue._command(String(data="GO"))
    for _ in range(20):
        queue._follower(String(data="PATH_ACCEPTED"))
        queue._follower(String(data="FOLLOWING_PATH"))

    assert queue.current_index == 0
    assert len(queue.goal.messages) == 1


def test_7_late_old_goal_reached_cannot_skip_the_new_goal():
    queue = queue_node([waypoint(1.0, 0.0), waypoint(2.0, 0.0)])
    queue._command(String(data="GO"))
    queue._follower(String(data="PATH_ACCEPTED"))
    queue._follower(String(data="GOAL_REACHED"))
    assert queue.current_index == 1
    assert queue.waiting_for_departure is True

    # A delayed state from goal 1 arrives before goal 2's path acknowledgement.
    queue._follower(String(data="GOAL_REACHED"))
    assert queue.current_index == 1
    assert len(queue.goal.messages) == 2


def test_8_planner_failure_keeps_current_goal_and_does_not_advance_queue():
    queue = queue_node([waypoint(1.0, 0.0), waypoint(2.0, 0.0)])
    queue._command(String(data="GO"))
    queue._follower(String(data="PATH_ACCEPTED"))

    for failure in ("NO_PATH", "NO_SAFE_PATH", "REPLAN_HOLD"):
        queue._follower(String(data=failure))

    assert queue.current_index == 0
    assert queue.goal.messages == [queue.queue[0]]
