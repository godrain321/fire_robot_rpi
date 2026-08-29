"""Stage 4: ADC-domain gas thresholds feeding the existing gas cost function.

Stage 4 adds no new cost algorithm. It only lets the existing CO cost /
blocked logic read its safe/blocked thresholds from ADC-domain parameters
(gas_input_mode="adc" -> gas_safe_adc / gas_blocked_adc) instead of the
legacy ppm ones, so an /mq135/filtered_adc scalar can be scored directly.
"""

import math

import numpy as np
import pytest

from inno_hazard.hazard_belief import (
    HazardBelief,
    HazardBeliefConfig,
    HazardGridGeometry,
)


def _belief(*, width=7, height=7, resolution=1.0, static=None, **cfg):
    geometry = HazardGridGeometry(width, height, resolution)
    static = np.zeros((height, width), bool) if static is None else static
    return HazardBelief(geometry, static, HazardBeliefConfig(**cfg))


ADC = dict(
    co_enabled=True, gas_input_mode="adc",
    gas_safe_adc=1000.0, gas_blocked_adc=3000.0,
    co_weight=8.0, co_power=2.0, gas_update_radius_m=0.0,
)


# --- A. adc mode selects the gas_*_adc thresholds ------------------------
def test_effective_thresholds_follow_gas_input_mode():
    adc = HazardBeliefConfig(gas_input_mode="adc",
                             gas_safe_adc=1000.0, gas_blocked_adc=3000.0)
    assert adc.gas_safe_threshold == 1000.0
    assert adc.gas_blocked_threshold == 3000.0

    legacy = HazardBeliefConfig(co_safe_ppm=100.0, co_blocked_ppm=1600.0)
    assert legacy.gas_safe_threshold == 100.0
    assert legacy.gas_blocked_threshold == 1600.0


# --- B. below safe -> zero gas cost -----------------------------------
def test_below_safe_adc_has_zero_gas_cost():
    item = _belief(**ADC)
    item.update_co_observation(2.0, 3.0, 800.0, 1.0)
    assert item.co_cost_map[3, 2] == 0.0
    assert not item.blocked_mask[3, 2]
    assert item.final_cost_map[3, 2] == item.config.base_cost


# --- C. between safe and blocked -> positive finite cost -------------
def test_middle_adc_has_positive_non_blocked_cost():
    item = _belief(**ADC)
    item.update_co_observation(2.0, 3.0, 2000.0, 1.0)
    # ratio = (2000-1000)/(3000-1000) = 0.5 -> 8 * 0.5**2 = 2.0
    assert item.co_cost_map[3, 2] == pytest.approx(2.0)
    assert not item.blocked_mask[3, 2]
    assert math.isfinite(item.final_cost_map[3, 2])
    assert item.final_cost_map[3, 2] == pytest.approx(item.config.base_cost + 2.0)


# --- D. at/above blocked -> blocked + inf, same policy as legacy -----
def test_at_blocked_adc_marks_cell_blocked():
    item = _belief(**ADC)
    item.update_co_observation(2.0, 3.0, 3100.0, 1.0)
    assert item.blocked_mask[3, 2]
    assert math.isinf(item.final_cost_map[3, 2])


# --- E. gas_update_radius_m = 0.0 -> only the robot cell changes -----
def test_only_the_robot_cell_is_affected():
    item = _belief(**ADC)
    before_cost = item.co_cost_map.copy()
    item.update_co_observation(2.0, 3.0, 2000.0, 1.0)
    changed = np.argwhere(item.co_cost_map != before_cost)
    assert changed.tolist() == [[3, 2]]
    assert int(item.co_observed_mask.sum()) == 1


# --- F. gas disabled -> byte-identical to a plain config ------------
def test_gas_disabled_matches_baseline_final_cost():
    static = np.zeros((7, 7), bool)
    base = _belief(static=static)  # legacy defaults, co disabled
    armed = _belief(static=static, gas_input_mode="adc",
                    gas_safe_adc=1000.0, gas_blocked_adc=3000.0)  # co still off
    # a disabled sensor must not invent anything even when handed a sample
    armed.update_co_observation(2.0, 3.0, 9999.0, 1.0)
    np.testing.assert_array_equal(base.final_cost_map, armed.final_cost_map)
    np.testing.assert_array_equal(base.blocked_mask, armed.blocked_mask)
    assert not armed.co_observed_mask.any()


# --- G. thermal path is untouched by the gas threshold mode ---------
def test_thermal_cost_is_independent_of_gas_mode():
    legacy = _belief()
    adc = _belief(gas_input_mode="adc",
                  gas_safe_adc=1000.0, gas_blocked_adc=3000.0)
    for grid in (legacy, adc):
        grid.update_temperature_observations([((2, 2), 52.0), ((3, 3), 61.0)], 1.0)
    np.testing.assert_array_equal(
        legacy.temperature_cost_map, adc.temperature_cost_map
    )
    np.testing.assert_array_equal(legacy.final_cost_map, adc.final_cost_map)
    np.testing.assert_array_equal(legacy.blocked_mask, adc.blocked_mask)


# --- H. invalid thresholds are rejected at construction ------------
def test_invalid_adc_thresholds_raise():
    with pytest.raises(ValueError):
        HazardBeliefConfig(gas_input_mode="adc",
                           gas_safe_adc=3000.0, gas_blocked_adc=1000.0)
    with pytest.raises(ValueError):
        HazardBeliefConfig(gas_input_mode="adc",
                           gas_safe_adc=2000.0, gas_blocked_adc=2000.0)
    with pytest.raises(ValueError):
        HazardBeliefConfig(gas_input_mode="millivolts")
    # legacy ppm validation still active
    with pytest.raises(ValueError):
        HazardBeliefConfig(co_safe_ppm=1600.0, co_blocked_ppm=100.0)


if __name__ == "__main__":
    raise SystemExit(pytest.main([__file__, "-q"]))
