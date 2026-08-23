"""Node-level contract test for ExitSwitchingNode._perform_peek's opposite-then-
any-safe-exit fallback (spec section 25), using the same fake-self pattern as
the other *_contract.py test files -- no live rclpy context.
"""

import json
from types import SimpleNamespace

import numpy as np

from inno_autonav.evacuation_planner import EvacuationPlanner, ExitSelectionConfig
from inno_autonav.exit_evaluator import ExitEvaluation, ExitEvaluationBatch, ExitHazardSnapshot
from inno_autonav.exit_switching_node import ExitSwitchingNode
from inno_autonav.exit_switching_orchestrator import PeekRequest
from inno_hazard.hazard_belief import HazardGridGeometry


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
