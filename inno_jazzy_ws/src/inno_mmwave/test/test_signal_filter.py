import math

import pytest

from inno_mmwave.signal_filter import (
    ACQUIRING,
    HOLDING,
    NO_TARGET,
    TRACKING,
    SignalFilterConfig,
    SingleTargetSignalFilter,
)


def feed(filter_item, timestamp, distance, raw_speed=0.0):
    return filter_item.update(
        timestamp=timestamp,
        detected=True,
        distance_m=distance,
        speed_mps=raw_speed,
        energy=100,
    )


def acquire(filter_item, start=0.0, centre=2.0, step=0.1):
    result = None
    offsets = (-0.01, 0.01, 0.0, 0.02, -0.02, 0.01, 0.0)
    for index, offset in enumerate(offsets):
        result = feed(
            filter_item,
            start + index * step,
            centre + offset,
        )
    assert result is not None and result.presence
    return result


def test_alternating_two_and_ten_metre_returns_lock_to_one_target():
    item = SingleTargetSignalFilter()
    outputs = []
    for index in range(30):
        distance = 2.0 + (0.02 if index % 4 == 0 else 0.0)
        if index % 3 == 1:
            distance = 10.0
        outputs.append(feed(item, index * 0.1, distance, raw_speed=7.5))

    present = [sample for sample in outputs if sample.presence]
    assert present
    assert all(1.8 < sample.distance_m < 2.2 for sample in present)
    assert any(sample.tracking_state == HOLDING for sample in present)
    assert present[-1].speed_mps == 0.0


def test_stationary_range_ignores_implausibly_large_raw_doppler_speed():
    item = SingleTargetSignalFilter()
    output = None
    for index in range(61):
        jitter = (0.015, -0.010, 0.005, -0.005)[index % 4]
        raw_speed = 7.8 if index % 2 else -8.2
        output = feed(item, index * 0.1, 3.0 + jitter, raw_speed)

    assert output is not None
    assert output.tracking_state == TRACKING
    assert output.presence
    assert output.distance_m == pytest.approx(3.0, abs=0.03)
    assert output.speed_mps == 0.0


@pytest.mark.parametrize('amplitude', [0.75, 3.8])
def test_repeated_plausible_doppler_detects_hand_motion_at_stable_range(amplitude):
    item = SingleTargetSignalFilter()
    acquire(item, centre=2.0)

    outputs = []
    for index in range(1, 16):
        now = 0.6 + index * 0.1
        distance = 2.0 + (0.01 if index % 2 else -0.01)
        raw_speed = amplitude if index % 2 else -amplitude
        outputs.append(feed(item, now, distance, raw_speed=raw_speed))

    assert any(abs(sample.speed_mps) > 0.0 for sample in outputs)
    assert max(sample.activity_percent for sample in outputs) >= 40.0
    assert outputs[-1].tracking_state == TRACKING


def test_isolated_plausible_doppler_spikes_do_not_declare_motion():
    item = SingleTargetSignalFilter()
    acquire(item, centre=2.0)

    speeds = (0.0, 0.8, 0.0, 0.0, -0.7, 0.0, 0.0, 0.0, 0.0)
    outputs = [
        feed(item, 0.7 + index * 0.1, 2.0, raw_speed=speed)
        for index, speed in enumerate(speeds)
    ]

    assert all(sample.speed_mps == 0.0 for sample in outputs)
    assert outputs[-1].activity_percent == 0.0


def test_persistent_human_scale_range_change_is_reported_as_motion():
    item = SingleTargetSignalFilter()
    acquire(item, centre=2.0)

    outputs = []
    # Walk away at 0.5 m/s.  Deliberately corrupt the raw Doppler field: the
    # accepted distance trend, rather than that field, must drive the result.
    for index in range(1, 31):
        now = 0.6 + index * 0.1
        distance = 2.0 + 0.5 * (now - 0.6)
        outputs.append(feed(item, now, distance, raw_speed=-7.0))

    moving = [sample for sample in outputs if abs(sample.speed_mps) > 0.0]
    assert moving
    assert moving[-1].speed_mps == pytest.approx(0.5, abs=0.12)
    assert outputs[-1].distance_m == pytest.approx(3.5, abs=0.35)


def test_isolated_jump_is_held_but_persistent_new_cluster_relocks():
    item = SingleTargetSignalFilter()
    acquire(item, centre=2.0)

    one_jump = feed(item, 0.7, 10.0)
    assert one_jump.presence
    assert one_jump.tracking_state == HOLDING
    assert one_jump.distance_m < 2.2

    relocked = None
    saw_relock = False
    for index in range(1, 26):
        relocked = feed(item, 0.7 + index * 0.1, 9.95 + 0.01 * (index % 2))
        saw_relock = saw_relock or relocked.reason == 'relocked'
    assert relocked is not None
    assert relocked.presence
    assert relocked.tracking_state == TRACKING
    assert saw_relock
    assert relocked.distance_m == pytest.approx(9.955, abs=0.05)


def test_presence_hold_and_expiry_use_elapsed_time():
    item = SingleTargetSignalFilter()
    acquire(item, centre=4.0)

    held = item.advance(2.0)
    assert held.presence
    assert held.tracking_state == HOLDING
    assert held.speed_mps == 0.0

    expired = item.advance(3.7)
    assert not expired.presence
    assert expired.tracking_state == NO_TARGET
    assert expired.distance_m == 0.0


def test_scattered_room_reflections_never_replace_a_confirmed_person():
    item = SingleTargetSignalFilter()
    acquire(item, centre=2.0)
    chaotic = (4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0, 11.0)
    outputs = [
        feed(item, 0.7 + index * 0.1, chaotic[index % len(chaotic)])
        for index in range(35)
    ]

    assert not any(sample.reason == 'relocked' for sample in outputs)
    assert all(
        1.8 < sample.distance_m < 2.2
        for sample in outputs if sample.presence
    )
    assert not outputs[-1].presence


def test_intermittent_zero_frames_do_not_erase_stable_acquisition_evidence():
    item = SingleTargetSignalFilter()
    result = None
    for index in range(6):
        result = feed(item, index * 0.2, 3.0 + 0.01 * (index % 2))
        if index < 5:
            missing = item.advance(index * 0.2 + 0.1)
            if result.presence:
                assert missing.presence
                assert missing.tracking_state == HOLDING

    assert result is not None and result.presence
    assert result.distance_m == pytest.approx(3.0, abs=0.03)


def test_invalid_ranges_do_not_become_targets_and_timestamps_are_monotonic():
    item = SingleTargetSignalFilter()
    below_spec = feed(item, 0.0, 0.4)
    assert below_spec.tracking_state == NO_TARGET

    first = feed(item, 1.0, 2.0)
    assert first.tracking_state == ACQUIRING
    with pytest.raises(ValueError, match='monotonic'):
        feed(item, 0.9, 2.0)
    with pytest.raises(ValueError, match='finite'):
        item.advance(math.nan)


def test_invalid_configuration_is_rejected():
    with pytest.raises(ValueError, match='support_ratio'):
        SingleTargetSignalFilter(SignalFilterConfig(
            acquire_min_support_ratio=1.1,
        ))
    with pytest.raises(ValueError):
        SingleTargetSignalFilter(SignalFilterConfig(
            motion_start_mps=0.05,
            motion_stop_mps=0.10,
        ))
