import math

import numpy as np

from inno_hazard.fire_localization import (
    FireEstimateState,
    FireLocalizationConfig,
    FireLocalizer,
    ThermalRay,
)
from inno_hazard.hazard_belief import HazardGridGeometry


def localizer(static=None, **changes):
    geometry = HazardGridGeometry(7, 7, 1.0)
    static = np.zeros((7, 7), bool) if static is None else static
    values = dict(
        enabled=True,
        possible_probability_threshold=0.2,
        likely_probability_threshold=0.4,
        confirm_probability_threshold=0.6,
        release_probability_threshold=0.5,
        candidate_probability_threshold=0.15,
        evidence_decay_per_second=0.0,
    )
    values.update(changes)
    return FireLocalizer(geometry, static, FireLocalizationConfig(**values))


def test_below_warning_has_no_thermal_evidence_and_probability_is_bounded():
    item = localizer()
    item.add_thermal_observation(
        "cold", 0, (0, 0, 0), (ThermalRay(39, ((1, 1),)),)
    )
    assert not item.thermal_fire_evidence.any()
    assert np.all((item.fire_probability >= 0) & (item.fire_probability <= 1))


def test_strong_repeated_crossing_rays_add_temporal_pose_support_and_confirm():
    item = localizer()
    observations = (
        ((0, 3, 0.0), ((1, 3), (2, 3), (3, 3))),
        ((3, 0, math.pi / 2), ((3, 1), (3, 2), (3, 3))),
        ((6, 3, math.pi), ((5, 3), (4, 3), (3, 3))),
        ((3, 6, -math.pi / 2), ((3, 5), (3, 4), (3, 3))),
        ((0, 3, 0.0), ((1, 3), (2, 3), (3, 3))),
        ((3, 0, math.pi / 2), ((3, 1), (3, 2), (3, 3))),
    )
    for index, (pose, cells) in enumerate(observations):
        item.add_thermal_observation(
            f"t{index}", index + 1, pose, (ThermalRay(80, cells),)
        )
    result = item.latest_result
    assert result.highest_probability_grid == (3, 3)
    assert result.distinct_pose_count >= 2
    assert result.state is FireEstimateState.CONFIRMED_FIRE_REGION


def test_evidence_decay_and_co_gradient_wall_stop_and_support_strength():
    static = np.zeros((7, 7), bool)
    static[3, 4] = True
    item = localizer(
        static, minimum_consecutive_co_rises=1,
        evidence_decay_per_second=0.1,
    )
    item.add_thermal_observation(
        "hot", 0, (1, 1, 0), (ThermalRay(60, ((2, 1),)),)
    )
    before = item.thermal_fire_evidence[1, 2]
    item.add_co_observation("base", 1, (1, 3, 0), 10, 25)
    assert item.thermal_fire_evidence[1, 2] < before
    item.add_co_observation(
        "rise", 2, (2, 3, 0), 30, 30,
        thermal_direction_supported=True,
    )
    assert item.co_gradient_evidence[3, 3] > 0
    assert item.co_gradient_evidence[3, 5] == 0
