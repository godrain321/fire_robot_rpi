"""Focused tests for stable mmWave status-console output."""

from pathlib import Path
import sys


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))

from inno_mmwave.status_console import (  # noqa: E402
    FILTERED_DISTANCE_TOPIC,
    FILTERED_PRESENCE_TOPIC,
    mode3_log_text,
    mode4_log_text,
    MODE_TITLES,
    StatusConsole,
    waypoint_log_text,
)


def test_console_uses_filtered_target_topics() -> None:
    assert FILTERED_PRESENCE_TOPIC == '/mmwave/human_presence'
    assert FILTERED_DISTANCE_TOPIC == '/mmwave/calibrated_distance_m'
    assert MODE_TITLES[3] == 'mmWave 사람 판별'
    assert MODE_TITLES[4] == '카메라 요구조자 판별'


def test_detection_distance_is_reported_to_one_decimal_metre() -> None:
    console = object.__new__(StatusConsole)
    console._distance_m = 2.04
    assert console._distance_m == 2.04


def test_mode2_named_waypoint_progress_is_operator_readable() -> None:
    assert waypoint_log_text('MODE2_RUNNING:1/3:w1') == (
        '[주행] w1로 이동 중 (1/3)'
    )
    assert waypoint_log_text('MODE2_REACHED:w1:SPACE_FOR:w5') == (
        '[도착] w1 도착 — Space를 누르면 w5 출발'
    )
    assert waypoint_log_text('MODE2_MISSION_COMPLETE:w6') == (
        '[완료] 선택한 웨이포인트 주행 완료'
    )


def test_mode3_states_are_translated_without_raw_english_codes() -> None:
    assert mode3_log_text('MODE3_READY:PRESS_SPACE') == (
        '[준비] Space를 누르면 가장 가까운 빨간 장애물을 검사합니다.'
    )
    assert mode3_log_text('MODE3_PERSON_CONFIRMED:MARKER_BLUE') == (
        '[결과] 사람 감지! — 해당 점을 파란색으로 변경'
    )
    assert mode3_log_text('MODE3_SENSOR_UNAVAILABLE:KEEP_RED') == (
        '[판정보류] mmWave 데이터 부족 또는 센서 연결 끊김 — 빨간색 유지'
    )


def test_mode4_states_are_translated_without_raw_english_codes() -> None:
    assert mode4_log_text('MODE4_CAMERA_YOLO_OBSERVING') == (
        '[판별] 정면 카메라 사람 판별 시작 — 현재 검사 중인 빨간 점에 적용'
    )
    assert mode4_log_text(
        'MODE4_DETECTION_SUMMARY:FRAMES=15:PERSON_FRAMES=9:'
        'VOTE_FRAMES=9:DETECTIONS=10:MAX_CONF=0.84'
    ) == (
        '[카메라 진단] 입력 15프레임, 사람 검출 9프레임, '
        '검사점 투표 9프레임, 최고 confidence 0.84'
    )
    assert mode4_log_text('MODE4_SURVIVOR_CONFIRMED:MARKER_BLUE') == (
        '[결과] 요구조자 감지! — 해당 점을 파란색으로 변경'
    )
