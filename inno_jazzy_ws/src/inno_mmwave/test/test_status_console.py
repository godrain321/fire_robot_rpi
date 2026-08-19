"""Focused tests for stable mmWave status-console output."""

from pathlib import Path
import sys


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))

from inno_mmwave.status_console import (  # noqa: E402
    FILTERED_DISTANCE_TOPIC,
    FILTERED_PRESENCE_TOPIC,
    MODE_TITLES,
    StatusConsole,
    waypoint_log_text,
)


def test_console_uses_filtered_target_topics() -> None:
    assert FILTERED_PRESENCE_TOPIC == '/mmwave/filtered_presence'
    assert FILTERED_DISTANCE_TOPIC == '/mmwave/filtered_distance_m'
    assert MODE_TITLES[3] == 'MMWAVE OBSTACLE INSPECTION'
    assert MODE_TITLES[4] == 'CAMERA + LIDAR SURVIVOR INSPECTION'


def test_detection_distance_is_reported_to_one_decimal_metre() -> None:
    console = object.__new__(StatusConsole)
    console._distance_m = 2.04
    assert console._detection_text() == 'DETECT, 2.0m'

    console._distance_m = 2.06
    assert console._detection_text() == 'DETECT, 2.1m'

    console._distance_m = None
    assert console._detection_text() == 'DETECT'


def test_mode2_named_waypoint_progress_is_operator_readable() -> None:
    assert waypoint_log_text('MODE2_RUNNING:1/3:w1') == (
        '[웨이포인트] w1 주행 중 (1/3)'
    )
    assert waypoint_log_text('MODE2_REACHED:w1:SPACE_FOR:w5') == (
        '[웨이포인트] w1 도착 - Space를 누르면 w5 출발'
    )
    assert waypoint_log_text('MODE2_MISSION_COMPLETE:w6') == (
        '[웨이포인트] 선택 경로 주행 완료'
    )
