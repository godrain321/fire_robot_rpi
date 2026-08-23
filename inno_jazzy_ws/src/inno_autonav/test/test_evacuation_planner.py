import json
import math

import pytest

from inno_autonav.evacuation_planner import (
    EvacuationFailureReason, EvacuationPlanner, ExitSelectionConfig,
    build_evacuation_decision, route_activation_decision,
)
from inno_autonav.exit_evaluator import (
    ExitEvaluation, ExitEvaluationBatch, ExitRejectionReason,
)


def evaluation(
    exit_id, *, accepted=True, reachable=True, status="unknown",
    length=10.0, risk=2.0,
):
    path_grid = ((0, 0), (1, 0)) if reachable else ()
    path_world = ((0.5, 0.5), (1.5, 0.5)) if reachable else ()
    return ExitEvaluation(
        exit_id, status, (2.0, 0.0),
        (1.5, 0.5) if reachable else None,
        (1, 0) if reachable else None,
        reachable, accepted, path_world, path_grid,
        length if reachable else None, risk if reachable else None,
        45.0 if reachable else None, None, 45.0 if reachable else None,
        None, 0.5 if reachable else None,
        () if accepted else (ExitRejectionReason.NO_PATH,),
        5.0, ("w1", "w2") if reachable else (),
    )


def batch(*items, revision=12, start=(0.5, 0.5)):
    return ExitEvaluationBatch(revision, "map", start, tuple(items), 5.0)


def selected(*items, config=None, risk_first=False):
    return EvacuationPlanner(config).plan(
        batch(*items), risk_first=risk_first
    )


def test_only_accepted_exit_is_selected_and_rejected_shorter_is_ignored():
    result = selected(
        evaluation("EXIT1", accepted=False, length=1.0),
        evaluation("EXIT2", length=20.0),
    )
    assert result.success and result.selected_exit_id == "EXIT2"


def test_default_order_is_length_then_risk_then_exit_id():
    assert selected(
        evaluation("EXIT1", length=10, risk=10),
        evaluation("EXIT2", length=12, risk=1),
    ).selected_exit_id == "EXIT1"
    assert selected(
        evaluation("EXIT1", length=10, risk=5),
        evaluation("EXIT2", length=10, risk=2),
    ).selected_exit_id == "EXIT2"
    assert selected(
        evaluation("EXIT2", length=10, risk=2),
        evaluation("EXIT1", length=10, risk=2),
    ).selected_exit_id == "EXIT1"


def test_confirmed_usable_subset_is_preferred_and_sorted():
    result = selected(
        evaluation("EXIT1", status="unknown", length=1),
        evaluation("EXIT3", status="usable", length=8, risk=1),
        evaluation("EXIT2", status="usable", length=8, risk=2),
    )
    assert result.selected_exit_id == "EXIT3"


def test_unknown_fallback_and_disabled_fallback():
    item = evaluation("EXIT1", status="unknown")
    assert selected(item).success
    config = ExitSelectionConfig(fallback_to_shortest_reachable_exit=False)
    result = selected(item, config=config)
    assert not result.success
    assert result.failure_reason is EvacuationFailureReason.NO_SAFE_EXIT


def test_all_rejected_no_exits_and_invalid_start_fail_explicitly():
    rejected = selected(evaluation("E", accepted=False))
    assert rejected.failure_reason is EvacuationFailureReason.NO_SAFE_EXIT
    empty = EvacuationPlanner().plan(batch())
    assert empty.failure_reason is EvacuationFailureReason.NO_EXITS_REGISTERED
    invalid = EvacuationPlanner().plan(
        batch(evaluation("E"), start=(math.nan, 0.0))
    )
    assert invalid.failure_reason is EvacuationFailureReason.INVALID_START_POSITION


def test_risk_first_reverses_primary_and_secondary_metrics():
    result = selected(
        evaluation("EXIT1", length=10, risk=10),
        evaluation("EXIT2", length=12, risk=1), risk_first=True,
    )
    assert result.selected_exit_id == "EXIT2"


def test_float_tolerance_produces_deterministic_exit_id_tie():
    result = selected(
        evaluation("EXIT2", length=10.0000002, risk=2.0000002),
        evaluation("EXIT1", length=10.0000001, risk=2.0000001),
    )
    assert result.selected_exit_id == "EXIT1"


def test_selected_path_metadata_and_hazard_revision_are_preserved():
    item = evaluation("EXIT2")
    result = EvacuationPlanner().plan(batch(item, revision=99))
    assert result.path_world == item.path_world
    assert result.path_grid == item.path_grid
    assert result.selected_evaluation.reference_waypoint_ids == ("w1", "w2")
    assert result.hazard_revision == 99


def test_candidate_and_excluded_ids_keep_future_api_without_switching_policy():
    source = batch(evaluation("E1"), evaluation("E2", length=20))
    planner = EvacuationPlanner()
    assert planner.plan(source, excluded_exit_ids=("E1",)).selected_exit_id == "E2"
    assert planner.plan(source, candidate_exit_ids=("E2",)).selected_exit_id == "E2"


def test_stage4_json_parser_and_manager_pipeline_select_without_activation():
    source = batch(evaluation("E1"), evaluation("E2", length=20), revision=7)
    payload = json.dumps(source.to_dict())
    plan, status, activated = build_evacuation_decision(
        payload, EvacuationPlanner(), activate=False, current_revision=7,
    )
    assert plan.selected_exit_id == "E1"
    assert status == "SELECTED_NOT_ACTIVATED"
    assert not activated


def test_activation_requires_same_current_hazard_revision_and_approach():
    plan = EvacuationPlanner().plan(batch(evaluation("E1"), revision=8))
    assert route_activation_decision(
        plan, activate=True, current_revision=7
    ) == ("EVALUATION_STALE", False)
    assert route_activation_decision(
        plan, activate=True, current_revision=8
    ) == ("ROUTE_ACTIVATED", True)


def test_invalid_stage4_payload_and_frame_are_rejected():
    with pytest.raises(ValueError):
        build_evacuation_decision("{}", EvacuationPlanner())
    payload = batch(evaluation("E1")).to_dict()
    payload["frame_id"] = "odom"
    with pytest.raises(ValueError):
        build_evacuation_decision(json.dumps(payload), EvacuationPlanner())
