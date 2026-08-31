import json
import math

import pytest

from inno_autonav.evacuation_demo import (
    group_leg_candidates,
    moving_priority_candidate,
    MovingCandidateTracker,
    build_next_exploration_decision,
    nearest_exit_obstacle_candidate,
    nearest_uninspected_candidate,
    parse_activation_response,
    parse_mode3_classification,
    parse_mode4_classification,
    startup_state,
)


def test_leg_candidates_pair_at_threshold_and_keep_single():
    assert group_leg_candidates([(0.0, 0.0), (0.70, 0.0), (2.0, 0.0)]) == (
        (0.35, 0.0), (2.0, 0.0)
    )


def test_leg_candidates_are_deterministic_non_overlapping_and_finite():
    values = [(0.6, 0.0), (0.0, 0.0), (0.3, 0.0), (math.nan, 1.0)]
    forward = group_leg_candidates(values)
    reverse = group_leg_candidates(reversed(values))
    assert forward == reverse
    assert len(forward) == 2


def test_leg_candidates_over_limit_remain_separate():
    assert group_leg_candidates([(0.0, 0.0), (0.71, 0.0)]) == (
        (0.0, 0.0), (0.71, 0.0)
    )


def test_moving_priority_uses_associated_red_and_suppression():
    assert moving_priority_candidate(
        [(2.05, 0.0)], [(1.0, 0.0), (2.0, 0.0)], (0.0, 0.0)
    ) == (2.0, 0.0)
    assert moving_priority_candidate(
        [(2.05, 0.0)], [(1.0, 0.0), (2.0, 0.0)], (0.0, 0.0), [(2.1, 0.0)]
    ) is None
from inno_autonav.exit_evaluator import ExitEvaluation, ExitEvaluationBatch


@pytest.mark.parametrize(
    "values, expected",
    [
        (("", "", "", "", False), "SEARCH_EXITS:SELECTING_MODE_5"),
        (("", "", "", "5:EVACUATION_DEMO", False), "SEARCH_EXITS:WAITING_FOR_HAZARD"),
        (("ACTIVE", "", "", "5:EVACUATION_DEMO", False), "SEARCH_EXITS:WAITING_FOR_EXIT_EVALUATOR"),
        (("ACTIVE", "READY", "", "5:EVACUATION_DEMO", False), "SEARCH_EXITS:WAITING_FOR_MANAGER"),
        (("ACTIVE", "READY", "READY", "5:EVACUATION_DEMO", False), "SEARCH_EXITS:WAITING_FOR_EVALUATION_SERVICE"),
        (("ACTIVE_THERMAL_ONLY", "READY", "READY", "5:EVACUATION_DEMO", True), "SEARCH_EXITS"),
        (("ACTIVE_STATIC_DYNAMIC_ONLY", "READY", "READY", "5:EVACUATION_DEMO", True), "SEARCH_EXITS"),
    ],
)
def test_startup_state(values, expected):
    assert startup_state(*values) == expected


def test_parse_activated_plan():
    result = parse_activation_response(True, json.dumps({
        "success": True,
        "selected_exit_id": "EXIT1",
        "activated": True,
    }))
    assert result.activated
    assert result.exit_id == "EXIT1"


@pytest.mark.parametrize(
    "success, payload, reason",
    [
        (False, "NOT_READY", "NOT_READY"),
        (True, "not-json", "INVALID_PLAN_RESPONSE"),
        (True, json.dumps({"success": False}), "NO_REACHABLE_EXIT"),
        (
            True,
            json.dumps({
                "success": True,
                "selected_exit_id": "EXIT2",
                "activated": False,
            }),
            "SELECTED_ROUTE_NOT_ACTIVATED",
        ),
    ],
)
def test_reject_unactivated_plan(success, payload, reason):
    result = parse_activation_response(success, payload)
    assert not result.activated
    assert result.reason == reason


def _evaluation(exit_id, length, x):
    return ExitEvaluation(
        exit_id, "unknown", (x, 0.0), (x - 0.5, 0.0), (int(x), 0),
        True, True, ((0.0, 0.0), (x - 0.5, 0.0)),
        ((0, 0), (int(x), 0)), length, length, 20.0, None, 20.0,
        None, 1.0, (), 1.0, ("w1",),
    )


def _batch_payload():
    return json.dumps(ExitEvaluationBatch(
        7,
        "map",
        (0.0, 0.0),
        (_evaluation("exit1", 3.0, 3.0), _evaluation("exit2", 8.0, 8.0)),
        1.0,
    ).to_dict())


def test_exit_exploration_selects_nearest_unchecked_exit_then_next():
    first = build_next_exploration_decision(_batch_payload(), ())
    assert first.success and first.target_exit_id == "exit1"
    assert json.loads(first.plan_payload)["activated"] is True

    second = build_next_exploration_decision(_batch_payload(), {"exit1"})
    assert second.success and second.target_exit_id == "exit2"

    complete = build_next_exploration_decision(
        _batch_payload(), {"exit1", "exit2"}
    )
    assert complete.complete


def test_only_candidate_near_exit_or_approach_is_selected():
    selected = nearest_exit_obstacle_candidate(
        [(10.0, 10.0), (2.2, 0.1), (1.8, 0.0)],
        (2.0, 0.0),
        (1.5, 0.0),
        0.5,
    )
    assert selected == (1.8, 0.0)


def test_closest_uninspected_candidate_is_selected_from_multiple_red_points():
    selected = nearest_uninspected_candidate(
        [(4.0, 0.0), (1.2, 0.0), (2.0, 0.0)],
        (0.0, 0.0),
        inspected_positions=[(1.0, 0.0)],
        suppression_radius_m=0.5,
    )
    assert selected == (2.0, 0.0)


def test_uninspected_candidate_rejects_invalid_robot_pose_and_radius():
    assert nearest_uninspected_candidate([(1.0, 0.0)], None) is None
    with pytest.raises(ValueError):
        nearest_uninspected_candidate([(1.0, 0.0)], (0.0, 0.0), (), 0.0)


@pytest.mark.parametrize(
    "payload, expected",
    [
        ("DYNAMIC_OBSTACLE:1.0,2.0", ("DYNAMIC_OBSTACLE", (1.0, 2.0))),
        ("PERSON:1.0,2.0", ("PERSON", (1.0, 2.0))),
        ("UNKNOWN:1.0,2.0", None),
        ("PERSON:not-a-point", None),
    ],
)
def test_parse_mode3_classification(payload, expected):
    assert parse_mode3_classification(payload) == expected


def test_stationary_lidar_jitter_is_not_a_moving_person_candidate():
    tracker = MovingCandidateTracker(
        minimum_displacement_m=0.20, minimum_observations=3
    )

    assert tracker.update([(1.00, 2.00)], 0.0) == ()
    assert tracker.update([(1.04, 2.01)], 0.2) == ()
    assert tracker.update([(0.98, 1.97)], 0.4) == ()


def test_three_temporally_associated_moving_points_create_one_candidate():
    tracker = MovingCandidateTracker(
        association_radius_m=0.75,
        minimum_displacement_m=0.20,
        minimum_observations=3,
    )

    tracker.update([(1.0, 2.0)], 0.0)
    tracker.update([(1.15, 2.0)], 0.2)
    detected = tracker.update([(1.35, 2.0)], 0.4)

    assert len(detected) == 1
    assert detected[0].position == (1.35, 2.0)
    assert detected[0].observations == 3
    assert detected[0].displacement_m == pytest.approx(0.35)
    # One physical track pre-empts Mode 5 only once.
    assert tracker.update([(1.55, 2.0)], 0.6) == ()


def test_stale_lidar_track_does_not_join_a_later_unrelated_point():
    tracker = MovingCandidateTracker(stale_timeout_sec=0.5)
    tracker.update([(0.0, 0.0)], 0.0)
    tracker.update([(0.1, 0.0)], 0.2)

    assert tracker.update([(0.3, 0.0)], 1.0) == ()


@pytest.mark.parametrize(
    "payload, target, expected",
    [
        ("NO_SURVIVOR", (1.0, 2.0), ("NO_SURVIVOR", None)),
        (
            "SURVIVOR:5.000,5.000,3;1.100,2.100,4",
            (1.0, 2.0),
            ("SURVIVOR", (1.1, 2.1)),
        ),
        ("SURVIVOR:invalid", (1.0, 2.0), None),
    ],
)
def test_parse_mode4_classification_selects_inspected_track(payload, target, expected):
    assert parse_mode4_classification(payload, target) == expected
