"""ROS-message contract tests for stamped source activation (no live node required)."""

import json
from types import MethodType, SimpleNamespace

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from std_msgs.msg import String

from inno_autonav.path_selector import PathSelectorCore
from inno_autonav.path_selector_node import PathSelectorNode
from inno_autonav.replan_supervisor import ActiveGoal


class Publisher:
    def __init__(self): self.messages = []
    def publish(self, message): self.messages.append(message)


def node(goal=ActiveGoal("EXIT1", (4.5, 0.5), 1)):
    value = SimpleNamespace(
        core=PathSelectorCore("WAYPOINT"), _active_goal=goal,
        _pending_activation=None, path_publisher=Publisher(),
    )
    for name in ("_matches_active_goal", "_complete_pending", "_apply"):
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
