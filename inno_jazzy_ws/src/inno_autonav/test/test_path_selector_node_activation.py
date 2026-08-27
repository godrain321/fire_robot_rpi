"""ROS-message contract tests for stamped source activation (no live node required)."""

import json
from types import MethodType, SimpleNamespace

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from std_msgs.msg import Int32, String

from inno_autonav.path_selector import PathSelectorCore
from inno_autonav.path_selector_node import PathSelectorNode
from inno_autonav.replan_supervisor import ActiveGoal


class Publisher:
    def __init__(self): self.messages = []
    def publish(self, message): self.messages.append(message)


class Logger:
    def __init__(self): self.messages = []
    def info(self, message): self.messages.append(("INFO", message))
    def warning(self, message): self.messages.append(("WARN", message))


def node(goal=ActiveGoal("EXIT1", (4.5, 0.5), 1)):
    logger = Logger()
    value = SimpleNamespace(
        core=PathSelectorCore("WAYPOINT"), _active_goal=goal,
        _pending_activation=None, path_publisher=Publisher(),
        _direct_goal_world=None, _drive_mode=1,
        _waypoint_failed_for_goal=False, _automatic_astar_fallback=False,
        direct_goal_modes={3, 4}, map_frame="map",
        get_logger=lambda: logger,
    )
    for name in (
        "_matches_active_goal", "_complete_pending", "_apply",
        "_activate_astar_fallback_if_ready",
    ):
        setattr(value, name, MethodType(getattr(PathSelectorNode, name), value))
    value._path_stamp_ns = PathSelectorNode._path_stamp_ns
    return value


def path(stamp, goal=(4.5, 0.5)):
    message = Path()
    message.header.stamp.sec = stamp // 1_000_000_000
    message.header.stamp.nanosec = stamp % 1_000_000_000
    pose = PoseStamped(); pose.pose.position.x, pose.pose.position.y = goal
    message.poses.append(pose)
    return message


def activation(stamp, goal=(4.5, 0.5)):
    return String(data=json.dumps({
        "mode": "A_STAR", "request_id": "1:EXIT1:1", "path_stamp_ns": stamp,
        "exit_id": "EXIT1", "goal_world": list(goal),
    }))


def test_cached_astar_is_relayed_unchanged_only_by_matching_activation():
    value = node(); candidate = path(101)
    PathSelectorNode._on_astar_path(value, candidate)
    assert not value.path_publisher.messages
    PathSelectorNode._on_mode(value, activation(101))
    assert value.path_publisher.messages == [candidate]
    assert value.core.mode.value == "A_STAR"


def test_result_before_path_waits_for_the_exact_stamped_path():
    value = node()
    PathSelectorNode._on_mode(value, activation(202))
    assert value.core.mode.value == "WAYPOINT"
    PathSelectorNode._on_astar_path(value, path(201))
    assert not value.path_publisher.messages
    candidate = path(202)
    PathSelectorNode._on_astar_path(value, candidate)
    assert value.path_publisher.messages == [candidate]


def test_old_exit_astar_path_is_rejected_after_goal_change():
    value = node(ActiveGoal("EXIT2", (4.5, 4.5), 2))
    PathSelectorNode._on_mode(value, activation(303, goal=(4.5, 4.5)))
    PathSelectorNode._on_astar_path(value, path(303, goal=(4.5, 0.5)))
    assert not value.path_publisher.messages
    assert value.core.mode.value == "WAYPOINT"


def test_empty_astar_path_is_never_activated():
    value = node(); empty = Path(); empty.header.stamp.nanosec = 404
    PathSelectorNode._on_mode(value, activation(404))
    PathSelectorNode._on_astar_path(value, empty)
    assert not value.path_publisher.messages
    assert value.core.mode.value == "WAYPOINT"


def test_mode3_direct_goal_selects_only_matching_astar_path():
    value = node()
    direct = PoseStamped()
    direct.header.frame_id = "map"
    direct.pose.position.x = 1.25
    direct.pose.position.y = 2.50
    PathSelectorNode._on_drive_mode(value, Int32(data=3))
    PathSelectorNode._on_direct_goal(value, direct)

    PathSelectorNode._on_astar_path(value, path(501, goal=(9.0, 9.0)))
    assert not value.path_publisher.messages
    candidate = path(502, goal=(1.25, 2.50))
    PathSelectorNode._on_astar_path(value, candidate)

    assert value.path_publisher.messages == [candidate]
    assert value.core.mode.value == "A_STAR"


def test_leaving_inspection_clears_direct_path_before_mode5_resume():
    value = node()
    value._drive_mode = 3
    value._direct_goal_world = (1.0, 2.0)
    value.core.set_mode("A_STAR")
    PathSelectorNode._on_drive_mode(value, Int32(data=5))

    assert value._direct_goal_world is None
    assert value.core.mode.value == "WAYPOINT"
    assert value.path_publisher.messages[-1].poses == []


def test_mode5_empty_waypoint_path_uses_matching_safe_astar_fallback():
    value = node()
    value._drive_mode = 5
    candidate = path(601)
    PathSelectorNode._on_waypoint_path(value, Path())
    assert not value.path_publisher.messages

    PathSelectorNode._on_astar_path(value, candidate)

    assert value.path_publisher.messages == [candidate]
    assert value.core.mode.value == "A_STAR"
    assert value._automatic_astar_fallback is True


def test_mode5_cached_astar_is_released_when_waypoint_failure_arrives_later():
    value = node()
    value._drive_mode = 5
    candidate = path(602)
    PathSelectorNode._on_astar_path(value, candidate)
    assert not value.path_publisher.messages

    PathSelectorNode._on_waypoint_path(value, Path())

    assert value.path_publisher.messages == [candidate]
    assert value.core.mode.value == "A_STAR"


def test_automatic_fallback_returns_to_a_recovered_waypoint_path():
    value = node()
    value._drive_mode = 5
    fallback = path(603)
    PathSelectorNode._on_waypoint_path(value, Path())
    PathSelectorNode._on_astar_path(value, fallback)
    recovered = path(604)

    PathSelectorNode._on_waypoint_path(value, recovered)

    assert value.path_publisher.messages == [fallback, recovered]
    assert value.core.mode.value == "WAYPOINT"
    assert value._automatic_astar_fallback is False


def test_empty_waypoint_path_does_not_auto_fallback_outside_mode5():
    value = node()
    PathSelectorNode._on_astar_path(value, path(605))
    PathSelectorNode._on_waypoint_path(value, Path())

    assert not value.path_publisher.messages
    assert value.core.mode.value == "WAYPOINT"
