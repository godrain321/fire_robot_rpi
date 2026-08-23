import numpy as np

from inno_autonav.safe_path_simplifier import (
    expanded_path,
    extract_direction_change_points,
    simplify_path_safely,
    supercover_line,
)


def test_supercover_includes_corner_side_cells():
    cells = supercover_line((0, 0), (2, 2))
    assert (1, 0) in cells
    assert (0, 1) in cells
    assert (1, 1) in cells
    assert cells[0] == (0, 0)
    assert cells[-1] == (2, 2)


def test_simplifier_rejects_shortcut_across_thermal_risk():
    data = np.zeros((7, 7), dtype=np.int8)
    data[3, 2:5] = 90
    raw = ((0, 3), (1, 3), (1, 2), (2, 1), (3, 1), (4, 1), (5, 2), (5, 3), (6, 3))
    result = simplify_path_safely(raw, data)
    assert result.safe
    touched = expanded_path(result.path)
    assert all(data[y, x] < 90 for x, y in touched)
    assert len(result.path) > 2


def test_simplifier_rejects_lethal_and_unknown_cells():
    data = np.zeros((5, 5), dtype=np.int8)
    data[2, 2] = 100
    raw = ((0, 2), (0, 1), (1, 0), (2, 0), (3, 0), (4, 1), (4, 2))
    result = simplify_path_safely(raw, data)
    assert result.safe
    assert (2, 2) not in expanded_path(result.path)


def test_open_space_simplifies_to_endpoints():
    data = np.zeros((5, 8), dtype=np.int8)
    raw = tuple((x, 2) for x in range(8))
    result = simplify_path_safely(raw, data)
    assert result.safe
    assert result.path == ((0, 2), (7, 2))


def test_exact_corner_shortcut_touching_wall_is_rejected():
    data = np.zeros((4, 4), dtype=np.int8)
    data[0, 1] = 100
    raw = ((0, 0), (0, 1), (0, 2), (1, 2), (2, 2))
    result = simplify_path_safely(raw, data)
    assert result.safe
    assert (1, 0) not in expanded_path(result.path)
    assert result.path != ((0, 0), (2, 2))


def test_direction_changes_and_endpoints_are_preserved():
    raw = ((0, 0), (1, 0), (2, 0), (2, 1), (2, 2), (3, 3))
    assert extract_direction_change_points(raw) == (
        (0, 0), (2, 0), (2, 2), (3, 3)
    )
    result = simplify_path_safely(raw, np.zeros((4, 4), dtype=np.int8))
    assert result.safe
    assert len(result.path) < len(raw)
    assert result.path[0] == raw[0]
    assert result.path[-1] == raw[-1]
