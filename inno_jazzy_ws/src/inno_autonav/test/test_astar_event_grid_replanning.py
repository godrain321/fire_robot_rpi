"""Event/dirty gating at astar_replanner's direct grid inputs."""

from types import MethodType, SimpleNamespace

import numpy as np

from inno_autonav.astar_replanner import AstarReplanner
from inno_autonav.grid_utils import MapGrid


def grid(data):
    values = np.asarray(data, dtype=np.int8)
    return MapGrid(
        width=values.shape[1], height=values.shape[0], resolution=1.0,
        origin_x=0.0, origin_y=0.0, origin_yaw=0.0, frame_id="map",
        data=values,
    )


def monitor(path=((0, 0), (1, 0), (2, 0))):
    value = SimpleNamespace(
        current_path_cells=list(path), path_replan_thermal_threshold=60,
        goal=object(), _replan_requested=False, _replan_reason="",
    )
    value._path_values = MethodType(AstarReplanner._path_values, value)
    value._remaining_path_cells = MethodType(
        AstarReplanner._remaining_path_cells, value
    )
    value._request_replan = MethodType(AstarReplanner._request_replan, value)
    return value


def test_thermal_change_away_from_path_does_not_request_replan():
    value = monitor()
    old = grid([[0, 0, 0], [0, 0, 0]])
    new = grid([[0, 0, 0], [0, 100, 0]])
    assert not AstarReplanner._thermal_path_risk_increased(value, old, new)


def test_thermal_threshold_crossing_on_path_requests_replan():
    value = monitor()
    old = grid([[0, 20, 0]])
    new = grid([[0, 60, 0]])
    assert AstarReplanner._thermal_path_risk_increased(value, old, new)


def test_thermal_100_on_path_always_requests_when_newly_blocked():
    value = monitor()
    old = grid([[0, 80, 0]])
    new = grid([[0, 100, 0]])
    assert AstarReplanner._thermal_path_risk_increased(value, old, new)


def test_new_dynamic_obstacle_on_path_requests_replan():
    value = monitor()
    old = grid([[0, 0, 0], [0, 0, 0]])
    new = grid([[0, 100, 0], [0, 0, 0]])
    assert AstarReplanner._new_dynamic_block_on_path(value, old, new)


def test_dynamic_obstacle_away_from_path_is_ignored():
    value = monitor()
    old = grid([[0, 0, 0], [0, 0, 0]])
    new = grid([[0, 0, 0], [0, 100, 0]])
    assert not AstarReplanner._new_dynamic_block_on_path(value, old, new)


def test_path_cells_behind_latest_tf_pose_are_ignored():
    value = monitor(path=((0, 0), (1, 0), (2, 0), (3, 0)))
    value.map_frame = "map"
    value.base_frame = "base_link"
    value.tf = SimpleNamespace(lookup_pose_2d=lambda *_: (2.5, 0.5, 0.0))
    old = grid([[0, 0, 0, 0]])
    new = grid([[100, 0, 0, 0]])
    assert not AstarReplanner._new_dynamic_block_on_path(value, old, new)


def test_many_fast_updates_coalesce_into_one_pending_request():
    value = monitor()
    for _ in range(8):
        AstarReplanner._request_replan(value, "THERMAL_PATH_RISK_INCREASED")
    assert value._replan_requested is True
    assert value._replan_reason == "THERMAL_PATH_RISK_INCREASED"
