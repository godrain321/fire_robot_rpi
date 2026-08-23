"""Actual rclpy/DDS integration tests for stamped PathSelector activation."""

import json
import time

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
import pytest
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

from inno_autonav.path_selector_node import PathSelectorNode


EXIT1 = (4.5, 0.5)
EXIT2 = (4.5, 4.5)


def transient_qos():
    qos = QoSProfile(depth=1)
    qos.reliability = ReliabilityPolicy.RELIABLE
    qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
    return qos


def make_path(stamp_ns, goal, marker):
    message = Path()
    message.header.frame_id = "map"
    message.header.stamp.sec, message.header.stamp.nanosec = divmod(
        stamp_ns, 1_000_000_000
    )
    first = PoseStamped(); first.pose.position.x = float(marker)
    last = PoseStamped(); last.pose.position.x, last.pose.position.y = goal
    message.poses = [first, last]
    return message


def plan(exit_id, goal, revision):
    return String(data=json.dumps({
        "success": True, "activated": True, "selected_exit_id": exit_id,
        "selected_approach_position_world": list(goal),
        "hazard_revision": revision,
    }))


def activation(mode, stamp, exit_id, goal):
    return String(data=json.dumps({
        "mode": mode, "request_id": f"1:{exit_id}:1",
        "path_stamp_ns": stamp, "exit_id": exit_id,
        "goal_world": list(goal),
    }))


class Harness:
    def __init__(self):
        self.selector = PathSelectorNode()
        self.probe = Node("stage8_dds_probe")
        self.waypoint = self.probe.create_publisher(Path, "/waypoint_path", transient_qos())
        self.astar = self.probe.create_publisher(Path, "/astar_path", transient_qos())
        self.plan = self.probe.create_publisher(String, "/evacuation/plan", transient_qos())
        self.mode = self.probe.create_publisher(String, "/path_selector/mode", 10)
        self.planned = []
        self.probe.create_subscription(
            Path, "/planned_path", self.planned.append, transient_qos()
        )
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.selector); self.executor.add_node(self.probe)
        self.spin(0.2)

    def spin(self, seconds=0.2):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            self.executor.spin_once(timeout_sec=0.01)

    def publish(self, publisher, message):
        publisher.publish(message); self.spin()

    def close(self):
        self.executor.remove_node(self.selector); self.executor.remove_node(self.probe)
        self.selector.destroy_node(); self.probe.destroy_node(); self.executor.shutdown()


@pytest.fixture
def harness():
    rclpy.init()
    value = Harness()
    try:
        yield value
    finally:
        value.close()
        if rclpy.ok(): rclpy.shutdown()


def test_scenario_a_waypoint_path_is_relayed_unchanged(harness):
    harness.publish(harness.plan, plan("EXIT1", EXIT1, 1))
    candidate = make_path(10, EXIT1, 11)
    harness.publish(harness.waypoint, candidate)
    assert harness.planned and harness.planned[-1] == candidate


@pytest.mark.parametrize("path_first", [True, False])
def test_astar_cross_topic_ordering_requires_both_path_and_activation(harness, path_first):
    harness.publish(harness.plan, plan("EXIT1", EXIT1, 1))
    candidate = make_path(20, EXIT1, 22)
    command = activation("A_STAR", 20, "EXIT1", EXIT1)
    first, second = ((harness.astar, candidate), (harness.mode, command)) if path_first else (
        (harness.mode, command), (harness.astar, candidate)
    )
    harness.publish(*first)
    assert not harness.planned
    harness.publish(*second)
    assert harness.planned and harness.planned[-1] == candidate


def test_wrong_stamp_and_empty_astar_are_not_relayed(harness):
    harness.publish(harness.plan, plan("EXIT1", EXIT1, 1))
    harness.publish(harness.mode, activation("A_STAR", 30, "EXIT1", EXIT1))
    harness.publish(harness.astar, make_path(31, EXIT1, 33))
    harness.publish(harness.astar, Path())
    assert not harness.planned


def test_late_exit1_astar_is_suppressed_after_exit2_plan(harness):
    harness.publish(harness.plan, plan("EXIT1", EXIT1, 1))
    harness.publish(harness.mode, activation("A_STAR", 40, "EXIT1", EXIT1))
    harness.publish(harness.plan, plan("EXIT2", EXIT2, 2))
    harness.publish(harness.astar, make_path(40, EXIT1, 44))
    assert not harness.planned
    assert harness.selector.core.mode.value == "WAYPOINT"
