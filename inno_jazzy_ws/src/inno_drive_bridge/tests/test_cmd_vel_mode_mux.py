from unittest.mock import patch

from geometry_msgs.msg import Twist

from inno_drive_bridge.cmd_vel_mode_mux import (
    CmdVelModeMux,
    command_source_for_modes,
)


class DummyPublisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


def test_operator_mode3_uses_isolated_greeting_source():
    assert command_source_for_modes(3, 3) == 3


def test_internal_mode3_during_integrated_run_keeps_navigation_source():
    assert command_source_for_modes(3, 2) == 2


def test_mode1_always_keeps_keyboard_source():
    assert command_source_for_modes(1, 1) == 1
    assert command_source_for_modes(1, 3) == 1


def test_zero_from_navigation_cannot_overwrite_active_greeting_command():
    node = object.__new__(CmdVelModeMux)
    node.mode = 3
    node.operator_mode = 3
    node.timeout = 0.35
    node.commands = {1: Twist(), 2: Twist(), 3: Twist()}
    node.commands[3].angular.z = 1.0
    node.received = {1: 0.0, 2: 0.0, 3: 10.0}
    node.output = DummyPublisher()

    # Model the publish callback at the same monotonic instant as the command.
    with patch(
        'inno_drive_bridge.cmd_vel_mode_mux.time.monotonic', return_value=10.0
    ):
        node._publish()

    assert node.output.messages[-1].angular.z == 1.0
