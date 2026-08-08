"""Focused tests for stable mmWave status-console output."""

from pathlib import Path
import sys


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))

from inno_mmwave.status_console import (  # noqa: E402
    FILTERED_DISTANCE_TOPIC,
    FILTERED_PRESENCE_TOPIC,
    StatusConsole,
)


def test_console_uses_filtered_target_topics() -> None:
    assert FILTERED_PRESENCE_TOPIC == '/mmwave/filtered_presence'
    assert FILTERED_DISTANCE_TOPIC == '/mmwave/filtered_distance_m'


def test_detection_distance_is_reported_to_one_decimal_metre() -> None:
    console = object.__new__(StatusConsole)
    console._distance_m = 2.04
    assert console._detection_text() == 'DETECT, 2.0m'

    console._distance_m = 2.06
    assert console._detection_text() == 'DETECT, 2.1m'

    console._distance_m = None
    assert console._detection_text() == 'DETECT'
