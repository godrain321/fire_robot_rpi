import math

import pytest

from inno_robot_bringup.mode3_greeting_spin import INTRO_TEXT, TimedOneTurn


def test_requested_korean_intro_text_is_exact():
    assert INTRO_TEXT == (
        '안녕하세요 저는 화재대피안내로봇입니다 '
        '안전한출구로 안내해드리겠습니다'
    )


def test_one_turn_runs_at_requested_speed_then_stops():
    turn = TimedOneTurn(1.0)
    turn.start(10.0)

    angular, finished = turn.command(10.0)
    assert angular == pytest.approx(1.0)
    assert not finished

    angular, finished = turn.command(10.0 + 2.0 * math.pi - 0.01)
    assert angular == pytest.approx(1.0)
    assert not finished

    angular, finished = turn.command(10.0 + 2.0 * math.pi)
    assert angular == 0.0
    assert finished
    assert not turn.active


def test_invalid_spin_speed_is_rejected():
    with pytest.raises(ValueError):
        TimedOneTurn(0.0)
