import math

import numpy as np
import pytest

from inno_autonav.exit_evaluator import (
    ExitEvaluationConfig, ExitEvaluator, ExitHazardSnapshot, ExitItem,
    ExitRejectionReason, ExitStatus, exit_evaluator_readiness, load_exit_registry,
    within_usable_confirmation_distance,
)
from inno_autonav.reference_waypoint_graph import (
    PlanningGridGeometry, ReferenceWaypoint, ReferenceWaypointGraphConfig,
    ReferenceWaypointGraphPlanner,
)
from inno_autonav.weighted_planner import WeightedPathResult, weighted_astar_search
from inno_hazard.hazard_belief import HazardGridGeometry


def snapshot(size=7, resolution=1.0, revision=7):
    shape = size, size
    geometry = HazardGridGeometry(size, size, resolution)
    final = np.ones(shape)
    temperature = np.full(shape, np.nan)
    co = np.full(shape, np.nan)
    false = np.zeros(shape, dtype=bool)
    return ExitHazardSnapshot(
        geometry, final, temperature, co, false, false, false,
        np.zeros(shape), false, false, false, revision, 60.0, 1600.0, 1.0,
    )


def changed(source, **updates):
    values = {
        name: np.array(getattr(source, name), copy=True)
        for name in (
            "final_cost", "temperature_c", "co_ppm", "observed_mask",
            "temperature_observed_mask", "co_observed_mask",
            "fire_probability", "static_obstacle_map",
            "dynamic_obstacle_map", "blocked_mask",
        )
    }
    values.update(updates)
    return ExitHazardSnapshot(
        source.geometry, values["final_cost"], values["temperature_c"],
        values["co_ppm"], values["observed_mask"],
        values["temperature_observed_mask"], values["co_observed_mask"],
        values["fire_probability"], values["static_obstacle_map"],
        values["dynamic_obstacle_map"], values["blocked_mask"],
        source.revision, source.temperature_blocked_c,
        source.co_blocked_ppm, source.base_cost,
    )


def cell_planner(view, start, goal):
    return weighted_astar_search(
        view.final_cost, start, goal, costs_are_traversal=True,
        use_traversal_cost=True, unknown_is_occupied=True,
    )


def evaluator(**config):
    return ExitEvaluator(ExitEvaluationConfig(**config), path_planner=cell_planner)


def evaluate(item, view=None, start=(0.5, 0.5), **config):
    return evaluator(**config).evaluate(
        item, start, snapshot=view or snapshot(), evaluated_at=3.0
    )


def test_reachable_registered_approach_and_basic_metrics():
    result = evaluate(ExitItem("EXIT1", (4.5, 0.5), (4.5, 0.5)))
    assert result.reachable and result.accepted
    assert result.approach_position_grid == (4, 0)
    assert result.path_length_m == 4.0
    assert result.accumulated_risk_cost == 0.0
    assert result.unknown_ratio == 1.0


def test_no_path_and_invalid_registered_approach_is_not_replaced():
    view = snapshot()
    static = np.array(view.static_obstacle_map, copy=True)
    static[:, 3] = True
    blocked = static.copy()
    costs = np.array(view.final_cost, copy=True)
    costs[blocked] = np.inf
    wall = changed(view, static_obstacle_map=static, blocked_mask=blocked,
                   final_cost=costs)
    result = evaluate(ExitItem("E", (5.5, 0.5), (5.5, 0.5)), wall)
    assert not result.reachable
    assert result.rejection_reasons == (ExitRejectionReason.NO_PATH,)

    static[0, 2] = True
    costs[0, 2] = np.inf
    invalid = changed(view, static_obstacle_map=static,
                      blocked_mask=static, final_cost=costs)
    result = evaluate(ExitItem("E", (4.5, 0.5), (2.5, 0.5)), invalid)
    assert result.rejection_reasons == (ExitRejectionReason.STATIC_OBSTACLE,)
    assert result.approach_position_grid == (2, 0)


def test_approach_search_chooses_reachable_lowest_path_cost_candidate():
    view = snapshot()
    costs = np.array(view.final_cost, copy=True)
    costs[:, 3:5] = 8.0
    result = evaluate(ExitItem("E", (4.5, 4.5), None),
                      changed(view, final_cost=costs), start=(0.5, 4.5))
    assert result.accepted
    assert result.approach_position_grid != (4, 4)
    assert math.dist(result.approach_position_world, (4.5, 4.5)) <= 1.0 + 1e-12


@pytest.mark.parametrize("kind,reason", [
    ("static", ExitRejectionReason.STATIC_OBSTACLE),
    ("dynamic", ExitRejectionReason.DYNAMIC_OBSTACLE),
    ("temperature", ExitRejectionReason.TEMPERATURE_LIMIT_EXCEEDED),
    ("co", ExitRejectionReason.CO_LIMIT_EXCEEDED),
])
def test_registered_approach_hard_rejection_reasons(kind, reason):
    view = snapshot()
    arrays = {name: np.array(getattr(view, name), copy=True) for name in (
        "final_cost", "temperature_c", "co_ppm", "temperature_observed_mask",
        "co_observed_mask", "static_obstacle_map", "dynamic_obstacle_map",
        "blocked_mask",
    )}
    cell = 0, 2
    if kind == "static":
        arrays["static_obstacle_map"][cell] = True
    elif kind == "dynamic":
        arrays["dynamic_obstacle_map"][cell] = True
    elif kind == "temperature":
        arrays["temperature_c"][cell] = 60.0
        arrays["temperature_observed_mask"][cell] = True
    else:
        arrays["co_ppm"][cell] = 1600.0
        arrays["co_observed_mask"][cell] = True
    arrays["blocked_mask"][cell] = True
    arrays["final_cost"][cell] = np.inf
    result = evaluate(
        ExitItem("E", (4.5, 0.5), (2.5, 0.5)), changed(view, **arrays)
    )
    assert result.rejection_reasons == (reason,)


def test_finite_soft_risk_remains_accepted_and_integrates_by_distance():
    view = snapshot()
    costs = np.full(view.final_cost.shape, 3.0)
    result = evaluate(
        ExitItem("E", (2.5, 0.5), (2.5, 0.5)),
        changed(view, final_cost=costs),
    )
    assert result.reachable and result.accepted
    assert result.path_length_m == 2.0
    assert result.accumulated_risk_cost == pytest.approx(4.0)
    limited = evaluate(
        ExitItem("E", (2.5, 0.5), (2.5, 0.5)),
        changed(view, final_cost=costs), dangerous_average_risk_cost=2.0,
    )
    assert limited.reachable and not limited.accepted
    assert ExitRejectionReason.PATH_RISK_COST_EXCEEDED in limited.rejection_reasons


def test_diagonal_path_length_and_trapezoidal_risk_numeric_parity():
    view = snapshot()
    costs = np.ones(view.final_cost.shape)
    costs[0, 0], costs[1, 1] = 2.0, 4.0
    diagonal = ExitEvaluator(
        ExitEvaluationConfig(),
        path_planner=lambda *_: WeightedPathResult(
            ((0, 0), (1, 1)), 1.0, 1
        ),
    )
    result = diagonal.evaluate(
        ExitItem("E", (1.5, 1.5), (1.5, 1.5)), (0.5, 0.5),
        snapshot=changed(view, final_cost=costs), evaluated_at=1.0,
    )
    assert result.path_length_m == pytest.approx(math.sqrt(2))
    assert result.accumulated_risk_cost == pytest.approx(2.0 * math.sqrt(2))


def test_unknown_ratio_and_observed_temperature_co_neighborhood():
    view = snapshot()
    observed = np.zeros(view.final_cost.shape, dtype=bool)
    observed[0, (0, 2, 4)] = True
    temp_mask = np.zeros_like(observed)
    co_mask = np.zeros_like(observed)
    temperature = np.full(view.final_cost.shape, np.nan)
    co = np.full(view.final_cost.shape, np.nan)
    temperature[1, 4], temp_mask[1, 4], observed[1, 4] = 59.0, True, True
    co[1, 4], co_mask[1, 4], observed[1, 4] = 1500.0, True, True
    result = evaluate(
        ExitItem("E", (4.5, 0.5), (4.5, 0.5)),
        changed(view, observed_mask=observed, temperature_c=temperature,
                co_ppm=co, temperature_observed_mask=temp_mask,
                co_observed_mask=co_mask),
    )
    assert result.accepted
    assert result.unknown_ratio == pytest.approx(2 / 5)
    assert result.exit_temperature_c == 59.0
    assert result.exit_co_ppm == 1500.0


def test_exit_neighborhood_hard_observation_rejects_but_path_is_reachable():
    view = snapshot()
    temperature = np.full(view.final_cost.shape, np.nan)
    mask = np.zeros(view.final_cost.shape, dtype=bool)
    temperature[1, 4], mask[1, 4] = 60.0, True
    result = evaluate(
        ExitItem("E", (4.5, 0.5), (4.5, 0.5)),
        changed(view, temperature_c=temperature,
                temperature_observed_mask=mask),
    )
    assert result.reachable and not result.accepted
    assert ExitRejectionReason.TEMPERATURE_LIMIT_EXCEEDED in result.rejection_reasons


@pytest.mark.parametrize("status,reason", [
    (ExitStatus.BLOCKED, ExitRejectionReason.EXIT_BLOCKED),
    (ExitStatus.DANGEROUS, ExitRejectionReason.EXIT_DANGEROUS),
    (ExitStatus.DANGER_EXPECTED, ExitRejectionReason.EXIT_DANGER_EXPECTED),
])
def test_known_status_rejects_before_path_planning(status, reason):
    calls = []
    checker = ExitEvaluator(
        ExitEvaluationConfig(), path_planner=lambda *args: calls.append(args)
    )
    result = checker.evaluate(
        ExitItem("E", (2.5, 0.5), (2.5, 0.5), status), (0.5, 0.5),
        snapshot=snapshot(), evaluated_at=1.0,
    )
    assert result.rejection_reasons == (reason,)
    assert not calls


def test_out_of_map_and_confirmation_helper():
    result = evaluate(ExitItem("E", (99.0, 0.5), None))
    assert result.rejection_reasons == (ExitRejectionReason.OUT_OF_MAP,)
    config = ExitEvaluationConfig()
    assert within_usable_confirmation_distance((0, 0), (3, 0), config)
    assert not within_usable_confirmation_distance((0, 0), (3.01, 0), config)


def test_batch_uses_one_revision_and_does_not_select_an_exit():
    batch = evaluator().evaluate_all(
        (ExitItem("E1", (2.5, 0.5), (2.5, 0.5)),
         ExitItem("E2", (3.5, 0.5), (3.5, 0.5))),
        (0.5, 0.5), snapshot=snapshot(revision=42), evaluated_at=9.0,
    )
    assert batch.hazard_revision == 42
    assert [item.exit_id for item in batch.evaluations] == ["E1", "E2"]
    assert "selected" not in batch.to_dict()


def test_hazard_not_ready_never_substitutes_a_zero_cost_map():
    view = snapshot()
    assert exit_evaluator_readiness(None, view.geometry, "ACTIVE") == "HAZARD_NOT_READY"
    assert exit_evaluator_readiness(
        view, view.geometry, "WAITING_FOR_THERMAL"
    ) == "HAZARD_NOT_READY:WAITING_FOR_THERMAL"
    assert exit_evaluator_readiness(
        view, view.geometry, "ACTIVE_THERMAL_ONLY"
    ) == "READY"
    assert exit_evaluator_readiness(
        view, view.geometry, "ACTIVE_STATIC_DYNAMIC_ONLY"
    ) == "READY"


@pytest.mark.parametrize("resolution,size", [(0.20, 30), (0.05, 120)])
def test_resolution_independent_physical_length_and_radius(resolution, size):
    view = snapshot(size=size, resolution=resolution)
    result = evaluate(
        ExitItem("E", (2.5, 0.5), (2.5, 0.5)), view,
        start=(0.5, 0.5),
    )
    assert result.accepted
    assert result.path_length_m == pytest.approx(2.0, abs=resolution * 1.5)


def test_reference_graph_result_metadata_and_fallback_are_preserved():
    view = snapshot(size=8)
    geometry = PlanningGridGeometry(1.0)
    graph = ReferenceWaypointGraphPlanner(
        (ReferenceWaypoint("w1", 1.5, 0.5),
         ReferenceWaypoint("w2", 3.5, 0.5)),
        ReferenceWaypointGraphConfig(
            neighbor_radius_m=2.1, connector_search_radius_m=2.0,
            waypoint_cost_radius_m=0.1,
        ),
    )

    def planner(snapshot_value, start, goal):
        return graph.plan(
            snapshot_value.final_cost, start, goal, geometry,
            snapshot_value.static_obstacle_map,
            costs_are_traversal=True, waypoint_frame_id="map",
        )

    graph_evaluator = ExitEvaluator(ExitEvaluationConfig(), path_planner=planner)
    result = graph_evaluator.evaluate(
        ExitItem("E", (4.5, 0.5), (4.5, 0.5)), (0.5, 0.5),
        snapshot=view, evaluated_at=1.0,
    )
    assert result.accepted
    assert result.reference_waypoint_ids == ("w1", "w2")

    far_graph = ReferenceWaypointGraphPlanner(
        (ReferenceWaypoint("w1", 7.5, 6.5),
         ReferenceWaypoint("w2", 7.5, 7.5)),
        ReferenceWaypointGraphConfig(connector_search_radius_m=1.1),
    )

    def fallback(snapshot_value, start, goal):
        return far_graph.plan(
            snapshot_value.final_cost, start, goal, geometry,
            snapshot_value.static_obstacle_map,
            costs_are_traversal=True, waypoint_frame_id="map",
        )

    result = ExitEvaluator(
        ExitEvaluationConfig(), path_planner=fallback
    ).evaluate(
        ExitItem("E", (4.5, 0.5), (4.5, 0.5)), (0.5, 0.5),
        snapshot=view, evaluated_at=1.0,
    )
    assert result.accepted and not result.reference_waypoint_ids


def test_exact_hazard_cost_reaches_evaluator_planner_without_reinterpretation():
    view = snapshot()
    costs = np.array(view.final_cost, copy=True)
    costs[0, 1] = 7.25
    exact = changed(view, final_cost=costs)
    seen = []

    def capturing_planner(snapshot_value, start, goal):
        seen.append(float(snapshot_value.final_cost[0, 1]))
        return weighted_astar_search(
            snapshot_value.final_cost, start, goal,
            costs_are_traversal=True, use_traversal_cost=True,
        )

    result = ExitEvaluator(
        ExitEvaluationConfig(), path_planner=capturing_planner
    ).evaluate(
        ExitItem("E", (2.5, 0.5), (2.5, 0.5)), (0.5, 0.5),
        snapshot=exact, evaluated_at=1.0,
    )
    assert result.accepted
    assert seen == [7.25]


def test_exit_registry_reuses_semantic_points_and_optional_approach(tmp_path):
    path = tmp_path / "semantic.yaml"
    path.write_text(
        "semantic_points:\n"
        "  exit1:\n"
        "    frame_id: map\n    x: 1.0\n    y: 2.0\n"
        "    approach: {x: 1.5, y: 2.5}\n"
        "  init: {frame_id: map, x: 0.0, y: 0.0}\n",
        encoding="utf-8",
    )
    items = load_exit_registry(str(path))
    assert items == (ExitItem("EXIT1", (1.0, 2.0), (1.5, 2.5)),)
