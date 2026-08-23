import math

import numpy as np
import pytest
import yaml

from inno_autonav.reference_waypoint_graph import (
    PlanningGridGeometry,
    ReferenceWaypoint,
    ReferenceWaypointGraphConfig,
    ReferenceWaypointGraphPlanner,
)
from inno_autonav.safe_path_simplifier import (
    expanded_path,
    simplify_path_safely,
)
from inno_autonav.waypoint_selection import (
    load_waypoint_document,
    named_waypoints_from_document,
)
from inno_autonav.weighted_planner import weighted_a_star_with_escape


def waypoint(name, x, y):
    return ReferenceWaypoint(name, float(x), float(y))


def plan(
    points, costs, start, goal, *, resolution=1.0, config=None,
    origin=(0.0, 0.0), static=None,
):
    geometry = PlanningGridGeometry(
        resolution, origin[0], origin[1], 0.0, "map"
    )
    static = (
        np.zeros_like(costs, dtype=bool) if static is None else static
    )
    planner = ReferenceWaypointGraphPlanner(points, config)
    result = planner.plan(
        costs, start, goal, geometry, static,
        waypoint_frame_id="map",
        unknown_is_occupied=True,
    )
    return planner, result


def test_waypoint_yaml_parsing_uses_shared_named_loader(tmp_path):
    document = {
        "version": 1,
        "frame_id": "map",
        "poses": {
            "w1": {"x": 1.2, "y": -2.3, "yaw": 0.4},
            "w2": {"x": 2.2, "y": -2.3, "yaw": 0.0},
        },
    }
    path = tmp_path / "waypoints.yaml"
    path.write_text(yaml.safe_dump(document), encoding="utf-8")
    loaded = named_waypoints_from_document(
        load_waypoint_document(path), "map"
    )
    assert [(item.name, item.x, item.y) for item in loaded] == [
        ("w1", 1.2, -2.3), ("w2", 2.2, -2.3),
    ]
    with pytest.raises(ValueError, match="odom"):
        named_waypoints_from_document(
            {"frame_id": "odom", "poses": document["poses"]}, "map"
        )


def test_graph_neighbors_use_world_distance_not_yaml_order():
    points = (
        waypoint("w3", 2, 0), waypoint("w1", 0, 0),
        waypoint("w4", 7, 0), waypoint("w2", 1, 0),
    )
    planner = ReferenceWaypointGraphPlanner(
        points, ReferenceWaypointGraphConfig(neighbor_radius_m=1.5)
    )
    edges = {
        frozenset((points[first].waypoint_id, points[second].waypoint_id))
        for first, second, _ in planner.candidate_edges
    }
    assert edges == {
        frozenset(("w1", "w2")), frozenset(("w2", "w3")),
    }


@pytest.mark.parametrize("resolution", [0.20, 0.05])
def test_graph_physical_neighbors_are_resolution_independent(resolution):
    points = tuple(waypoint(f"w{i}", float(i), 1.0) for i in range(1, 4))
    width = int(math.ceil(5.0 / resolution))
    height = int(math.ceil(2.0 / resolution))
    costs = np.zeros((height, width))
    start = (int(0.5 / resolution), int(1.0 / resolution))
    goal = (int(3.5 / resolution), int(1.0 / resolution))
    planner, result = plan(
        points, costs, start, goal, resolution=resolution,
        config=ReferenceWaypointGraphConfig(
            neighbor_radius_m=1.5,
            connector_search_radius_m=0.6,
            waypoint_cost_radius_m=0.10,
        ),
    )
    assert len(planner.candidate_edges) == 2
    assert result.used_reference_graph
    assert result.reference_waypoint_ids == ("w1", "w2", "w3")


def test_wall_removes_distance_candidate_from_valid_graph_route():
    points = (waypoint("left", 0.125, 0.625), waypoint("right", 1.125, 0.625))
    costs = np.zeros((5, 6))
    costs[:, 2] = 100
    planner, result = plan(
        points, costs, (0, 2), (4, 2), resolution=0.25,
        config=ReferenceWaypointGraphConfig(
            neighbor_radius_m=1.1, connector_search_radius_m=0.2,
            fallback_to_cell_astar=False,
        ),
    )
    assert len(planner.candidate_edges) == 1
    assert not result.path
    assert result.reason == "no safe reference graph route"


def test_start_connector_tries_second_reachable_candidate():
    points = (
        waypoint("nearest_blocked", 1.5, 1.5),
        waypoint("second", 2.5, 2.5), waypoint("goal_anchor", 4.5, 2.5),
    )
    costs = np.zeros((6, 7))
    costs[1, 1] = 100
    _, result = plan(
        points, costs, (1, 0), (5, 2),
        config=ReferenceWaypointGraphConfig(
            neighbor_radius_m=2.1, connector_search_radius_m=3.0,
            connector_candidate_count=3,
        ),
    )
    assert result.used_reference_graph
    assert result.reference_waypoint_ids[0] == "second"


def test_goal_connector_tries_reachable_candidate():
    points = (
        waypoint("start_anchor", 1.5, 2.5),
        waypoint("reachable", 3.5, 2.5),
        waypoint("nearest_blocked", 4.5, 1.5),
    )
    costs = np.zeros((6, 7))
    costs[1, 4] = 100
    _, result = plan(
        points, costs, (0, 2), (5, 1),
        config=ReferenceWaypointGraphConfig(
            neighbor_radius_m=2.1, connector_search_radius_m=3.0,
            connector_candidate_count=3,
        ),
    )
    assert result.used_reference_graph
    assert result.reference_waypoint_ids[-1] == "reachable"


def diamond_fixture(risk_weight=1.0):
    points = (
        waypoint("w1", 1.5, 2.5), waypoint("w2", 2.5, 1.5),
        waypoint("w3", 2.5, 3.5), waypoint("w4", 3.5, 2.5),
    )
    config = ReferenceWaypointGraphConfig(
        neighbor_radius_m=1.5, connector_search_radius_m=1.1,
        connector_candidate_count=1, waypoint_risk_weight=risk_weight,
    )
    return points, config


def test_graph_route_assembles_expected_reference_branch():
    points, config = diamond_fixture(risk_weight=0.0)
    _, result = plan(points, np.zeros((5, 5)), (0, 2), (4, 2), config=config)
    assert result.used_reference_graph
    assert result.reference_waypoint_ids == ("w1", "w2", "w4")


def test_risk_weight_selects_longer_safe_reference_branch():
    points = (
        waypoint("start", 1.5, 3.5), waypoint("upper", 3.5, 1.5),
        waypoint("lower_a", 2.5, 4.5), waypoint("lower_b", 4.5, 4.5),
        waypoint("goal", 5.5, 3.5),
    )
    costs = np.zeros((7, 7))
    costs[1, 3] = 95
    _, result = plan(
        points, costs, (0, 3), (6, 3),
        config=ReferenceWaypointGraphConfig(
            neighbor_radius_m=3.0, connector_search_radius_m=1.1,
            connector_candidate_count=1, waypoint_risk_weight=1.0,
        ),
    )
    assert result.used_reference_graph
    assert "upper" not in result.reference_waypoint_ids
    assert result.reference_waypoint_ids == (
        "start", "lower_a", "lower_b", "goal"
    )


def test_graph_failure_falls_back_to_stage1_cell_planner():
    points = (waypoint("far1", 20, 20), waypoint("far2", 21, 20))
    costs = np.zeros((5, 7))
    _, result = plan(
        points, costs, (0, 2), (6, 2),
        config=ReferenceWaypointGraphConfig(
            connector_search_radius_m=0.5,
            fallback_to_cell_astar=True,
        ),
    )
    assert result.path
    assert not result.used_reference_graph
    assert "cell A* fallback: path found" in result.reason


def test_disabled_graph_matches_stage1_planner_exactly():
    costs = np.zeros((5, 7))
    static = np.zeros_like(costs, dtype=bool)
    planner, result = plan(
        (), costs, (0, 2), (6, 2), static=static,
        config=ReferenceWaypointGraphConfig(
            enabled=False, fallback_to_cell_astar=False
        ),
    )
    expected = weighted_a_star_with_escape(costs, (0, 2), (6, 2), static)
    assert not planner.config.enabled
    assert result.path == expected.path
    assert result.total_cost == expected.total_cost
    assert not result.used_reference_graph


def test_blocked_start_uses_escape_before_reference_graph():
    points = (waypoint("w1", 2.5, 2.5), waypoint("w2", 4.5, 2.5))
    costs = np.zeros((5, 7))
    costs[1:4, :2] = 100
    _, result = plan(points, costs, (0, 2), (6, 2))
    assert result.path
    assert result.escape_path
    assert not result.used_reference_graph
    assert "start requires escape" in result.reason


def test_final_cell_path_is_continuous():
    points, config = diamond_fixture()
    _, result = plan(points, np.zeros((5, 5)), (0, 2), (4, 2), config=config)
    assert result.used_reference_graph
    assert result.path[0] == (0, 2)
    assert result.path[-1] == (4, 2)
    assert all(
        max(abs(second[0] - first[0]), abs(second[1] - first[1])) == 1
        for first, second in zip(result.path, result.path[1:])
    )


def test_graph_path_passes_existing_safe_simplifier():
    points, config = diamond_fixture()
    costs = np.zeros((5, 5))
    costs[2, 2] = 100
    _, result = plan(points, costs, (0, 2), (4, 2), config=config)
    assert result.used_reference_graph
    simplified = simplify_path_safely(result.path, costs)
    assert simplified.safe
    assert simplified.path[0] == result.path[0]
    assert simplified.path[-1] == result.path[-1]
    assert all(costs[y, x] < 100 for x, y in expanded_path(simplified.path))


def test_graph_and_astar_share_exact_hazard_traversal_costs():
    points, config = diamond_fixture()
    costs = np.ones((5, 5), dtype=float)
    costs[1, 2] = 40.0
    planner = ReferenceWaypointGraphPlanner(points, config)
    result = planner.plan(
        costs, (0, 2), (4, 2), PlanningGridGeometry(1.0),
        np.zeros_like(costs, dtype=bool), waypoint_frame_id="map",
        costs_are_traversal=True,
    )
    assert result.used_reference_graph
    assert "w2" not in result.reference_waypoint_ids
    assert "w3" in result.reference_waypoint_ids
