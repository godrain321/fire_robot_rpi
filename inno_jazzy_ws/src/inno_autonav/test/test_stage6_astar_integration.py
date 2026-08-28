"""Test 6/7 from the follow-up fix: ReplanSupervisorCore's decision is the only
thing that may cause astar_replanner._plan() to run once periodic_replanning is
disabled (Stage 6 enabled). Wires the pure supervisor core's output straight into
the same fake-astar_replanner harness used by test_astar_periodic_replanning.py.
"""

from types import MethodType, SimpleNamespace

from geometry_msgs.msg import PoseStamped
import numpy as np

from inno_autonav.astar_replanner import AstarReplanner
from inno_autonav.event_replanning import EventReplanningConfig
from inno_autonav.exit_evaluator import ExitHazardSnapshot
from inno_autonav.replan_supervisor import ActiveGoal, ReplanSupervisorCore
from inno_hazard.hazard_belief import HazardGridGeometry


GOAL = ActiveGoal("EXIT1", (4.5, 0.5), 1)
SAFE_PATH_WORLD = [(0.5, 0.5), (1.5, 0.5), (2.5, 0.5), (3.5, 0.5), (4.5, 0.5)]


class Publisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


def fake_astar_node():
    calls = []
    clock = SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=10_000_000_000))
    value = SimpleNamespace(
        goal=None, _dirty=False, _replan_requested=False,
        _replan_reason="", _planning=False,
        goal_duplicate_tolerance=0.01, periodic_replanning_enabled=False,
        replan_rate=1.0, _last_plan=0.0,
        map_frame="map", state_publisher=Publisher(),
        get_logger=lambda: SimpleNamespace(
            error=lambda *a, **k: None, debug=lambda *a, **k: None
        ),
        get_clock=lambda: clock,
    )
    value.calls = calls
    value._plan = lambda reason: calls.append(reason)
    value._state = MethodType(AstarReplanner._state, value)
    value._same_goal = MethodType(AstarReplanner._same_goal, value)
    return value


def deliver_goal(astar, world_xy):
    message = PoseStamped()
    message.header.frame_id = "map"
    message.pose.position.x, message.pose.position.y = world_xy
    AstarReplanner._goal_callback(astar, message)


def snapshot(size=8, revision=1, **overrides):
    shape = size, size
    geometry = HazardGridGeometry(size, size, 1.0)
    values = {
        "final_cost": np.ones(shape),
        "temperature_c": np.full(shape, np.nan),
        "co_ppm": np.full(shape, np.nan),
        "observed_mask": np.zeros(shape, dtype=bool),
        "temperature_observed_mask": np.zeros(shape, dtype=bool),
        "co_observed_mask": np.zeros(shape, dtype=bool),
        "fire_probability": np.zeros(shape),
        "static_obstacle_map": np.zeros(shape, dtype=bool),
        "dynamic_obstacle_map": np.zeros(shape, dtype=bool),
        "blocked_mask": np.zeros(shape, dtype=bool),
    }
    values.update(overrides)
    return ExitHazardSnapshot(
        geometry, values["final_cost"], values["temperature_c"], values["co_ppm"],
        values["observed_mask"], values["temperature_observed_mask"],
        values["co_observed_mask"], values["fire_probability"],
        values["static_obstacle_map"], values["dynamic_obstacle_map"],
        values["blocked_mask"], revision, 60.0, 1600.0, 1.0,
    )


def bootstrap(core, astar):
    core.on_active_goal(GOAL)
    core.on_planned_path(SAFE_PATH_WORLD)
    core.on_hazard_snapshot(snapshot(revision=0))
    baseline = core.tick((0.5, 0.5), 0.0)
    assert baseline.status["state"] == "PATH_VALID"
    # Timer keeps running with periodic replanning disabled; nothing has
    # requested a plan yet.
    for _ in range(3):
        AstarReplanner._timer_callback(astar)
    assert astar.calls == []


def test_6_safe_revision_change_never_reaches_astar_plan():
    core = ReplanSupervisorCore(EventReplanningConfig())
    astar = fake_astar_node()
    bootstrap(core, astar)

    out = core.on_hazard_snapshot(snapshot(revision=1))
    assert out.publish_goal is None  # supervisor decided nothing needs to change

    # No message was ever handed to astar_replanner's goal callback, and its own
    # timer (periodic replanning disabled) never calls _plan either.
    for _ in range(3):
        AstarReplanner._timer_callback(astar)
    assert astar.calls == []


def test_7_unsafe_revision_change_causes_exactly_one_plan_via_same_goal():
    core = ReplanSupervisorCore(EventReplanningConfig())
    astar = fake_astar_node()
    bootstrap(core, astar)

    dynamic = np.zeros((8, 8), dtype=bool)
    dynamic[0, 2] = True
    out = core.on_hazard_snapshot(snapshot(revision=1, dynamic_obstacle_map=dynamic))
    assert out.hold is True
    assert out.publish_goal == GOAL.approach_world

    # This is exactly what replan_supervisor_node._publish() does with a
    # non-None publish_goal. A distinct first goal plans immediately.
    deliver_goal(astar, out.publish_goal)
    assert astar.calls == ["NEW_GOAL"]

    # Extra timer ticks (e.g. while the new path is being validated) must not
    # add a second, unrequested A* run.
    for _ in range(3):
        AstarReplanner._timer_callback(astar)
    assert astar.calls == ["NEW_GOAL"]
