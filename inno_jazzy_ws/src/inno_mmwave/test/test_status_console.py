"""Focused tests for stable mmWave status-console output."""

from pathlib import Path
import sys

from std_msgs.msg import Bool, String


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))

from inno_mmwave.status_console import (  # noqa: E402
    DYNAMIC_OBSTACLE_TOPIC,
    FILTERED_DISTANCE_TOPIC,
    FILTERED_PRESENCE_TOPIC,
    MODE_TITLES,
    StatusConsole,
    waypoint_log_text,
)


def test_console_has_three_explicit_operator_modes() -> None:
    assert MODE_TITLES == {
        1: 'KEYBOARD',
        2: 'WAYPOINT ONLY',
        3: 'RESCUE + DYNAMIC AVOIDANCE',
    }


def test_console_uses_filtered_target_topics() -> None:
    assert FILTERED_PRESENCE_TOPIC == '/mmwave/filtered_presence'
    assert FILTERED_DISTANCE_TOPIC == '/mmwave/filtered_distance_m'
    assert DYNAMIC_OBSTACLE_TOPIC == '/dynamic_obstacle_detected'


def test_waypoint_log_only_formats_arrival_and_completion_events() -> None:
    assert waypoint_log_text('REACHED:2/5') == (
        '[웨이포인트] 2번 도착 (2/5)'
    )
    assert waypoint_log_text('MISSION_COMPLETE') == '[웨이포인트] 주행 완료'
    assert waypoint_log_text('RUNNING:3/5') is None


def test_mode_two_and_three_report_waypoint_arrival_only() -> None:
    console = object.__new__(StatusConsole)
    console._mode = 2
    console._waypoint_state = None
    lines = []
    console._write = lines.append

    console._on_waypoint_state(String(data='RUNNING:1/3'))
    console._on_waypoint_state(String(data='REACHED:1/3'))
    console._mode = 3
    console._on_waypoint_state(String(data='REACHED:2/3'))

    assert lines == [
        '[웨이포인트] 1번 도착 (1/3)',
        '[웨이포인트] 2번 도착 (2/3)',
    ]


def test_mode_three_dynamic_obstacle_log_is_transition_only() -> None:
    console = object.__new__(StatusConsole)
    console._mode = 3
    console._dynamic_detected = False
    lines = []
    console._write = lines.append

    console._on_dynamic_obstacle(Bool(data=True))
    console._on_dynamic_obstacle(Bool(data=True))
    console._on_dynamic_obstacle(Bool(data=False))

    assert lines == [
        '[동적장애물] 감지됨',
        '[동적장애물] 감지 해제',
    ]


def test_detection_distance_is_reported_to_one_decimal_metre() -> None:
    console = object.__new__(StatusConsole)
    console._distance_m = 2.04
    assert console._detection_text() == '[사람] 감지됨, 거리 약 2.0m'

    console._distance_m = 2.06
    assert console._detection_text() == '[사람] 감지됨, 거리 약 2.1m'

    console._distance_m = None
    assert console._detection_text() == '[사람] 감지됨'
