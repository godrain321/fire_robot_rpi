"""Node-level contract test for ExitSwitchingNode._perform_peek's opposite-then-
any-safe-exit fallback (spec section 25), using the same fake-self pattern as
the other *_contract.py test files -- no live rclpy context.
"""

import json
from pathlib import Path
from types import SimpleNamespace

import numpy as np
from std_msgs.msg import Int32, String

from inno_autonav.evacuation_planner import EvacuationPlanner, ExitSelectionConfig
from inno_autonav.exit_evaluator import ExitEvaluation, ExitEvaluationBatch, ExitHazardSnapshot
from inno_autonav.exit_switching import ExitSwitchingConfig
from inno_autonav.exit_switching_node import ExitSwitchingNode
from inno_autonav.exit_switching_orchestrator import (
    ExitSwitchingCore, ForcedProximitySwitch, PeekRequest,
)
from inno_autonav.replan_supervisor import ActiveGoal
from inno_hazard.hazard_belief import HazardGridGeometry


def test_proximity_trigger_ros_default_is_declared_as_double_array():
    source = (
        Path(__file__).parents[1] / "inno_autonav" / "exit_switching_node.py"
    ).read_text(encoding="utf-8")

    assert '"demo_force_trigger_waypoint_positions": [0.0, 0.0]' in source


def test_mode3_pause_preserves_exit2_and_ignores_inspection_plan():
    core = ExitSwitchingCore(
        ExitSwitchingConfig(),
        {"EXIT2": (14.0, -7.0), "EXIT3": (1.0, -12.0)},
        ForcedProximitySwitch(
            "EXIT2", "EXIT3", ((12.471, -6.803),), 1.0,
        ),
    )
    core.on_active_goal(ActiveGoal("EXIT2", (14.0, -7.0), 1))
    value = SimpleNamespace(
        core=core,
        _pause_drive_modes={3, 4},
        _configured_enabled=True,
        _exit_ids=frozenset({"EXIT2", "EXIT3"}),
        _apply=lambda _output: None,
    )

    ExitSwitchingNode._on_drive_mode(value, Int32(data=3))
    assert core.enabled is False
    assert core.active_goal.exit_id == "EXIT2"

    inspection_plan = json.dumps({
        "success": True,
        "activated": True,
        "selected_exit_id": "MODE3_INSPECTION",
        "selected_approach_position_world": [5.0, -10.0],
        "hazard_revision": 2,
    })
    ExitSwitchingNode._on_plan(value, String(data=inspection_plan))
    assert core.active_goal.exit_id == "EXIT2"

    ExitSwitchingNode._on_drive_mode(value, Int32(data=5))
    assert core.enabled is True
    out = core.tick((12.471, -6.803), 10.0)
    assert out.switch_request is not None
    assert out.switch_request.candidate_exit_ids == ("EXIT3",)


class EvaluationClient:
    def __init__(self, response):
        self.response = response

    def wait_for_service(self, timeout_sec):
        del timeout_sec
        return True

    def call(self, request, timeout_sec):
        del request, timeout_sec
        return self.response


def _evaluation(exit_id, *, accepted, path_length):
    return ExitEvaluation(
        exit_id, "unknown", (float(path_length) + 1.0, 0.0), (1.0, 0.5), (1, 0),
        accepted, accepted, ((0.5, 0.5), (float(path_length), 0.5)),
        ((0, 0), (int(path_length), 0)), float(path_length), 0.0, 40.0, None,
        40.0, None, 0.0, (), 1.0, (),
    )


def _batch_payload():
    from std_srvs.srv import Trigger

    response = Trigger.Response()
    response.success = True
    response.message = json.dumps(ExitEvaluationBatch(
        4, "map", (0.5, 0.5),
        (
            _evaluation("EXIT1", accepted=False, path_length=1.0),  # rejected candidate
            _evaluation("EXIT2", accepted=True, path_length=2.0),  # current, excluded
            _evaluation("EXIT3", accepted=True, path_length=3.0),  # only safe fallback
        ),
        1.0,
    ).to_dict())
    return response


def node():
    shape = (8, 8)
    snapshot = ExitHazardSnapshot(
        HazardGridGeometry(8, 8, 1.0), np.ones(shape), np.full(shape, np.nan),
        np.full(shape, np.nan), np.zeros(shape, dtype=bool), np.zeros(shape, dtype=bool),
        np.zeros(shape, dtype=bool), np.zeros(shape), np.zeros(shape, dtype=bool),
        np.zeros(shape, dtype=bool), np.zeros(shape, dtype=bool), 4, 60.0, 1600.0, 1.0,
    )
    value = SimpleNamespace(
        snapshot=snapshot,
        evaluation_client=EvaluationClient(_batch_payload()),
        evaluation_timeout=1.0,
        map_frame="map",
        peek_planner=EvacuationPlanner(ExitSelectionConfig()),
        get_logger=lambda: SimpleNamespace(error=lambda *a, **k: None),
    )
    return value


def test_opposite_only_candidate_fails_falls_back_to_any_safe_exit():
    value = node()
    request = PeekRequest(
        "sustained_route_cost_increase", "EXIT2", ("EXIT2",), ("EXIT1",), False,
    )
    result = ExitSwitchingNode._perform_peek(value, request)
    assert result.success is True
    assert result.selected_exit_id == "EXIT3"
    assert result.used_fallback is True


def test_opposite_candidate_accepted_does_not_need_fallback():
    value = node()
    request = PeekRequest(
        "sustained_route_cost_increase", "EXIT2", ("EXIT2",), ("EXIT3",), False,
    )
    result = ExitSwitchingNode._perform_peek(value, request)
    assert result.success is True
    assert result.selected_exit_id == "EXIT3"
    assert result.used_fallback is False


def test_no_snapshot_yet_fails_the_peek_without_calling_the_service():
    value = node()
    value.snapshot = None
    request = PeekRequest("reason", "EXIT2", ("EXIT2",), None, False)
    result = ExitSwitchingNode._perform_peek(value, request)
    assert result.success is False
