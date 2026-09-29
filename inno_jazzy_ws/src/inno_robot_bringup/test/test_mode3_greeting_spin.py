from unittest.mock import patch

import pytest
from std_msgs.msg import Empty, Int32

from inno_robot_bringup.mode3_greeting_spin import (
    INTRO_TEXT,
    Mode3GreetingSpin,
    TURN_ANGLE_RADIANS,
    TimedOneTurn,
)


class DummyPublisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


class DummyLogger:
    def warning(self, _message):
        pass

    def info(self, _message):
        pass


def test_requested_korean_intro_text_is_exact():
    assert INTRO_TEXT == (
        '안녕하세요 저는 화재대피안내로봇입니다 '
        '안전한출구로 안내해드리겠습니다'
    )


def test_450_degree_turn_runs_at_requested_speed_then_stops():
    turn = TimedOneTurn(1.0)
    turn.start(10.0)

    angular, finished = turn.command(10.0)
    assert angular == pytest.approx(1.0)
    assert not finished

    angular, finished = turn.command(10.0 + TURN_ANGLE_RADIANS - 0.01)
    assert angular == pytest.approx(1.0)
    assert not finished

    angular, finished = turn.command(10.0 + TURN_ANGLE_RADIANS)
    assert angular == 0.0
    assert finished
    assert not turn.active


def test_invalid_spin_speed_is_rejected():
    with pytest.raises(ValueError):
        TimedOneTurn(0.0)


def test_greeting_uses_a_dedicated_velocity_topic():
    from pathlib import Path

    source = (
        Path(__file__).parents[1]
        / 'inno_robot_bringup'
        / 'mode3_greeting_spin.py'
    )
    text = source.read_text(encoding='utf-8')
    assert "Twist, '/cmd_vel_mode3_demo', 10" in text
    assert "Twist, '/cmd_vel_auto', 10" not in text


def test_delayed_previous_cancel_cannot_stop_new_mode3_request():
    node = object.__new__(Mode3GreetingSpin)
    node.operator_mode = 1
    node._request_pending = False
    node.turn = TimedOneTurn(1.0)
    node.velocity_publisher = DummyPublisher()
    node.status_publisher = DummyPublisher()
    node._reported_complete = True
    node.get_logger = lambda: DummyLogger()
    play_count = []
    node._play_intro = lambda: play_count.append(1)

    # Reproduce the observed cross-topic order: request, old cancel, then the
    # operator-mode update.  Motion must begin only after Mode 3 is confirmed.
    node._on_request(Empty())
    assert node._request_pending
    assert not node.turn.active
    node._on_cancel(Empty())
    assert node._request_pending

    with patch(
        'inno_robot_bringup.mode3_greeting_spin.time.monotonic',
        return_value=10.0,
    ):
        node._on_operator_mode(Int32(data=3))

    assert node.turn.active
    assert play_count == [1]
    assert node.velocity_publisher.messages[-1].angular.z == 1.0

    # A delayed cancel from the old autonomy still cannot stop Mode 3.
    node._on_cancel(Empty())
    assert node.turn.active

    # The keyboard's real c/s cancellation ends by selecting operator Mode 1.
    node._on_operator_mode(Int32(data=1))
    assert not node.turn.active
