import pytest

from inno_mmwave.c4001_protocol import C4001Measurement
from inno_mmwave.human_detection import (
    HumanDetectionConfig,
    HumanPresenceDetector,
)


def target(distance_m=1.5, energy=10000, speed_mps=0.0):
    return C4001Measurement(
        target_count=1,
        target_id=1,
        distance_m=distance_m,
        speed_mps=speed_mps,
        energy=energy,
        raw_frame='$DFDMD,test*',
    )


def no_target():
    return C4001Measurement(
        target_count=0,
        target_id=None,
        distance_m=None,
        speed_mps=None,
        energy=None,
        raw_frame='$DFDMD,0, , , , , , *',
    )


def detector():
    return HumanPresenceDetector(HumanDetectionConfig())


def test_defaults_are_the_field_tuned_values():
    config = HumanDetectionConfig()
    assert config.calibration_scale == pytest.approx(1.0)
    assert config.calibration_offset_m == pytest.approx(-0.1)
    assert (config.range_min_m, config.range_max_m) == pytest.approx((0.6, 6.0))
    assert config.energy_threshold == 3000
    assert (config.confirm_frames, config.clear_frames) == (3, 6)


def test_raw_distance_is_calibrated_without_mutating_measurement():
    measurement = target(distance_m=1.5)
    result = detector().update(measurement)
    assert measurement.distance_m == pytest.approx(1.5)
    assert result.calibrated_distance_m == pytest.approx(1.4)


def test_presence_requires_three_consecutive_matching_frames():
    item = detector()
    assert not item.update(target()).presence
    assert not item.update(target()).presence
    assert item.update(target()).presence


def test_low_energy_and_out_of_range_samples_do_not_confirm():
    item = detector()
    for measurement in (
        target(1.5, energy=2999),
        target(0.65, energy=10000),  # calibrated to 0.55 m
        target(6.2, energy=10000),   # calibrated to 6.1 m
    ):
        result = item.update(measurement)
        assert not result.sample_matches
        assert not result.presence


def test_six_clear_frames_are_required_after_presence():
    item = detector()
    for _ in range(3):
        assert item.update(target()).sample_matches
    assert item.presence
    for _ in range(5):
        assert item.update(no_target()).presence
    assert not item.update(no_target()).presence


def test_reset_clears_presence_and_distance():
    item = detector()
    for _ in range(3):
        item.update(target())
    result = item.reset()
    assert not result.presence
    assert result.calibrated_distance_m is None


@pytest.mark.parametrize(
    'config',
    [
        HumanDetectionConfig(calibration_scale=0.0),
        HumanDetectionConfig(range_min_m=2.0, range_max_m=1.0),
        HumanDetectionConfig(energy_threshold=-1),
        HumanDetectionConfig(confirm_frames=0),
        HumanDetectionConfig(clear_frames=0),
    ],
)
def test_invalid_configuration_is_rejected(config):
    with pytest.raises(ValueError):
        HumanPresenceDetector(config)
