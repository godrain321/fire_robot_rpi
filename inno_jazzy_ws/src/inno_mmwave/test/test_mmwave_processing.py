"""Headless tests for the standalone tuning pipeline."""

import pytest

from inno_mmwave.mmwave_processing import (
    DistanceProcessor,
    FilterType,
    HumanCandidateDetector,
    HumanTuningSettings,
    ProcessingSettings,
)
from inno_mmwave.mmwave_tuning_gui import SensorSettings


def test_tuning_defaults_match_field_settings():
    sensor = SensorSettings()
    software = ProcessingSettings()
    human = HumanTuningSettings()

    assert (sensor.min_range_cm, sensor.max_range_cm) == (60, 600)
    assert sensor.threshold == 20
    assert sensor.fretting_enabled
    assert software.scale == pytest.approx(1.0)
    assert software.offset_m == pytest.approx(-0.1)
    assert software.filter_type is FilterType.NONE
    assert software.filter_size == 5
    assert software.ema_alpha == pytest.approx(0.3)
    assert software.outlier_threshold_m == pytest.approx(0.0)
    assert (human.range_min_m, human.range_max_m) == pytest.approx((0.6, 6.0))
    assert human.energy_threshold == 3000
    assert (human.confirm_frames, human.clear_frames) == (3, 6)


def test_raw_calibrated_and_filtered_values_remain_separate():
    processor = DistanceProcessor(
        ProcessingSettings(
            scale=1.1,
            offset_m=0.05,
            filter_type=FilterType.NONE,
        )
    )
    result = processor.process(1.4, target_valid=True)
    assert result.raw_range_m == 1.4
    assert result.calibrated_range_m == pytest.approx(1.59)
    assert result.filtered_range_m == pytest.approx(1.59)


def test_no_target_and_invalid_range_never_enter_filter():
    processor = DistanceProcessor(
        ProcessingSettings(
            offset_m=0.0,
            filter_type=FilterType.MOVING_AVERAGE,
            filter_size=3,
        )
    )
    assert not processor.process(0.0, target_valid=False).accepted
    assert not processor.process(0.0, target_valid=True).accepted
    result = processor.process(1.5, target_valid=True)
    assert result.filtered_range_m == pytest.approx(1.5)


def test_outlier_is_excluded_and_zero_disables_rejection():
    processor = DistanceProcessor(ProcessingSettings(outlier_threshold_m=0.5))
    assert processor.process(1.5, target_valid=True).accepted
    rejected = processor.process(3.8, target_valid=True)
    assert not rejected.accepted
    assert rejected.rejection_reason == "OUTLIER"
    processor.apply_settings(ProcessingSettings(outlier_threshold_m=0.0))
    assert processor.process(3.8, target_valid=True).accepted


@pytest.mark.parametrize(
    ("filter_type", "expected"),
    [
        (FilterType.MEDIAN, 2.0),
        (FilterType.MOVING_AVERAGE, 8.0 / 3.0),
    ],
)
def test_window_filters(filter_type, expected):
    processor = DistanceProcessor(
        ProcessingSettings(
            offset_m=0.0, filter_type=filter_type, filter_size=3
        )
    )
    for value in (1.0, 2.0):
        processor.process(value, target_valid=True)
    result = processor.process(5.0, target_valid=True)
    assert result.filtered_range_m == pytest.approx(expected)


def test_ema_and_filter_change_reset_state():
    processor = DistanceProcessor(
        ProcessingSettings(
            offset_m=0.0, filter_type=FilterType.EMA, ema_alpha=0.5
        )
    )
    processor.process(1.0, target_valid=True)
    assert processor.process(3.0, target_valid=True).filtered_range_m == pytest.approx(2.0)
    processor.apply_settings(
        ProcessingSettings(
            offset_m=0.0, filter_type=FilterType.EMA, ema_alpha=0.25
        )
    )
    assert processor.process(4.0, target_valid=True).filtered_range_m == pytest.approx(4.0)


def test_processing_validation():
    with pytest.raises(ValueError):
        ProcessingSettings(filter_size=0)
    with pytest.raises(ValueError):
        ProcessingSettings(ema_alpha=0.0)
    with pytest.raises(ValueError):
        ProcessingSettings(outlier_threshold_m=-0.1)


def test_human_candidate_uses_confirm_and_clear_frames():
    detector = HumanCandidateDetector(
        HumanTuningSettings(
            range_min_m=1.2,
            range_max_m=1.8,
            energy_threshold=100,
            confirm_frames=3,
            clear_frames=2,
        )
    )
    for _ in range(2):
        assert not detector.update(
            target_count=1, filtered_range_m=1.5, energy=120
        )
    assert detector.update(target_count=1, filtered_range_m=1.5, energy=120)
    assert detector.update(target_count=0, filtered_range_m=None, energy=None)
    assert not detector.update(target_count=0, filtered_range_m=None, energy=None)


def test_sensor_settings_use_official_uart_commands_without_save():
    settings = SensorSettings(
        min_range_cm=30,
        max_range_cm=240,
        threshold=15,
        fretting_enabled=False,
    )
    assert settings.volatile_commands() == (
        "sensorStop",
        "setRunApp 1",
        "setRange 0.3 2.4",
        "setThrFactor 15",
        "setMicroMotion 0",
        "sensorStart",
    )
    assert "saveConfig" not in settings.volatile_commands()


def test_sensor_settings_validate_official_speed_mode_ranges():
    with pytest.raises(ValueError):
        SensorSettings(min_range_cm=29, max_range_cm=240)
    with pytest.raises(ValueError):
        SensorSettings(min_range_cm=30, max_range_cm=239)
    with pytest.raises(ValueError):
        SensorSettings(min_range_cm=300, max_range_cm=240)
    with pytest.raises(ValueError):
        SensorSettings(min_range_cm=30, max_range_cm=2001)
    with pytest.raises(ValueError):
        SensorSettings(threshold=65536)
