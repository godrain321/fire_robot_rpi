"""Headless tests for the mmWave GUI data model and chart helpers."""

from pathlib import Path
import sys


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))

from inno_mmwave.mmwave_gui import (  # noqa: E402
    FILTERED_DISTANCE_TOPIC,
    FILTERED_PRESENCE_TOPIC,
    FILTERED_SPEED_TOPIC,
    RAW_ENERGY_TOPIC,
    MOTION_ACTIVITY_TOPIC,
    format_distance_m,
    HistoryPoint,
    MobilityState,
    TelemetryStore,
    chart_coordinates,
    format_duration,
    nice_upper_bound,
    normalize_mobility_state,
    normalize_sensor_state,
)


class FakeClock:
    def __init__(self, start: float = 100.0) -> None:
        self.now = start

    def __call__(self) -> float:
        return self.now

    def advance(self, seconds: float) -> None:
        self.now += seconds


def test_state_normalization_uses_public_contract() -> None:
    assert normalize_mobility_state("MOVING") is MobilityState.MOVING
    assert normalize_mobility_state("still-monitor") is MobilityState.STILL_MONITOR
    assert normalize_mobility_state("assist check") is MobilityState.ASSIST_CHECK
    assert normalize_mobility_state("unknown future state") is MobilityState.NO_TARGET
    assert normalize_sensor_state("ONLINE / ttyAMA0") == "ONLINE"
    assert normalize_sensor_state("error: serial timeout") == "ERROR"
    assert normalize_sensor_state("") == "CONNECTING"


def test_store_overrides_classifier_when_offline_or_no_target() -> None:
    clock = FakeClock()
    store = TelemetryStore(clock=clock, stale_timeout_sec=2.0)

    assert store.snapshot().mobility_state is MobilityState.SENSOR_OFFLINE

    store.update_sensor_state("ONLINE")
    store.update_presence(True)
    store.update_distance(2.43)
    snapshot = store.snapshot()
    assert snapshot.online
    assert snapshot.presence
    assert snapshot.distance_m == 2.43
    assert snapshot.mobility_state is MobilityState.STILL_MONITOR

    store.update_mobility_state("MOVING")
    assert store.snapshot().mobility_state is MobilityState.MOVING

    store.update_presence(False)
    assert store.snapshot().mobility_state is MobilityState.NO_TARGET

    clock.advance(2.1)
    stale = store.snapshot()
    assert not stale.online
    assert not stale.presence
    assert stale.sensor_state == "OFFLINE"
    assert stale.mobility_state is MobilityState.SENSOR_OFFLINE


def test_store_prunes_history_to_window_and_rejects_invalid_values() -> None:
    clock = FakeClock()
    store = TelemetryStore(clock=clock, history_window_sec=5.0)
    store.update_presence(True)
    store.update_distance(1.0)
    clock.advance(4.0)
    store.update_distance(2.0)
    clock.advance(2.0)
    store.update_distance(float("nan"))

    snapshot = store.snapshot()
    assert [point.value for point in snapshot.distance_history] == [2.0]
    assert snapshot.distance_m is None


def test_store_does_not_graph_no_target_or_zero_distance() -> None:
    clock = FakeClock()
    store = TelemetryStore(clock=clock)

    store.update_distance(9.0)
    assert store.snapshot().distance_history == ()

    store.update_presence(True)
    store.update_distance(2.04)
    clock.advance(0.1)
    store.update_distance(0.0)

    snapshot = store.snapshot()
    assert snapshot.distance_m is None
    assert [point.value for point in snapshot.distance_history] == [2.04]


def test_motion_activity_is_clamped_and_kept_as_the_single_chart_series() -> None:
    clock = FakeClock()
    store = TelemetryStore(clock=clock, history_window_sec=5.0)
    store.update_motion_activity(15.0)
    clock.advance(1.0)
    store.update_motion_activity(150.0)

    snapshot = store.snapshot()
    assert snapshot.motion_activity_percent == 100.0
    assert [point.value for point in snapshot.motion_history] == [15.0, 100.0]


def test_classifier_topics_do_not_keep_dead_uart_online() -> None:
    clock = FakeClock()
    store = TelemetryStore(clock=clock, stale_timeout_sec=2.0)
    store.update_sensor_state("ONLINE")
    store.update_presence(True)

    for _ in range(5):
        clock.advance(0.5)
        store.update_mobility_state("STILL_MONITOR")
        store.update_still_duration(clock.now - 100.0)

    snapshot = store.snapshot()
    assert not snapshot.online
    assert snapshot.sensor_state == "OFFLINE"
    assert snapshot.mobility_state is MobilityState.SENSOR_OFFLINE


def test_chart_coordinates_clip_values_and_ignore_old_samples() -> None:
    points = chart_coordinates(
        (
            HistoryPoint(65.0, 5.0),
            HistoryPoint(70.0, -2.0),
            HistoryPoint(85.0, 6.0),
            HistoryPoint(100.0, 15.0),
        ),
        now=100.0,
        window_sec=30.0,
        value_min=0.0,
        value_max=12.0,
        x=10.0,
        y=20.0,
        width=300.0,
        height=120.0,
    )
    assert points == ((10.0, 140.0), (160.0, 80.0), (310.0, 20.0))


def test_display_helpers() -> None:
    assert format_duration(9.8) == "00:09"
    assert format_duration(75) == "01:15"
    assert format_duration(3671) == "01:01:11"
    assert nice_upper_bound([0.12, 0.31], minimum=0.5) == 0.5
    assert nice_upper_bound([21, 330], minimum=100.0) == 500.0
    assert format_distance_m(2.04) == "2.0 m"
    assert format_distance_m(2.06, include_unit=False) == "2.1"
    assert format_distance_m(0.0) == "—"


def test_gui_uses_filtered_target_topics_and_motion_activity_topic() -> None:
    assert FILTERED_PRESENCE_TOPIC == "/mmwave/filtered_presence"
    assert FILTERED_DISTANCE_TOPIC == "/mmwave/filtered_distance_m"
    assert FILTERED_SPEED_TOPIC == "/mmwave/filtered_speed_mps"
    assert RAW_ENERGY_TOPIC == "/mmwave/raw/energy_raw"
    assert MOTION_ACTIVITY_TOPIC == "/mmwave/motion_activity"
