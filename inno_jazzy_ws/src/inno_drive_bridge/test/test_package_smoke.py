import pytest

from inno_drive_bridge.cmd_vel_mode_mux import command_source_for_mode


def test_drive_package_imports():
    import inno_drive_bridge.cmd_vel_mode_mux  # noqa: F401
    import inno_drive_bridge.cmdvel_to_esp32_serial  # noqa: F401


def test_mode_two_and_three_share_only_the_autonomous_input():
    assert command_source_for_mode(1) == 1
    assert command_source_for_mode(2) == 2
    assert command_source_for_mode(3) == 2
    with pytest.raises(ValueError, match='1, 2, or 3'):
        command_source_for_mode(4)
