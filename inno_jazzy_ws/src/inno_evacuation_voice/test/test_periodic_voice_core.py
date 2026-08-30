import pytest

from inno_evacuation_voice.periodic_voice_core import (
    PlaybackDecision, PeriodicVoiceCore, stop_child_process,
)


def core(**kwargs):
    return PeriodicVoiceCore(now=0.0, **kwargs)


def test_inactive_does_not_request_playback():
    assert core().due(100.0) is PlaybackDecision.NONE


def test_mode5_plays_immediately_then_uses_elapsed_time():
    scheduler = core()
    scheduler.set_drive_mode(5, 10.0)
    assert scheduler.due(10.0) is PlaybackDecision.PLAY
    scheduler.playback_started(10.0)
    scheduler.playback_finished()
    assert scheduler.due(16.9) is PlaybackDecision.NONE
    assert scheduler.due(17.0) is PlaybackDecision.PLAY


def test_busy_period_is_skipped_without_overlap():
    scheduler = core(activation_mode='always')
    scheduler.playback_started(0.0)
    assert scheduler.due(7.0) is PlaybackDecision.BUSY
    assert scheduler.due(13.9) is PlaybackDecision.NONE
    scheduler.playback_finished()
    assert scheduler.due(14.0) is PlaybackDecision.PLAY


def test_mode_change_stops_and_reentry_restarts_immediately():
    scheduler = core()
    scheduler.set_drive_mode(5, 1.0)
    scheduler.playback_started(1.0)
    assert scheduler.set_drive_mode(3, 2.0) is True
    assert scheduler.due(100.0) is PlaybackDecision.NONE
    scheduler.set_drive_mode(5, 101.0)
    assert scheduler.due(101.0) is PlaybackDecision.PLAY


def test_autonomy_cancel_stops_repetition():
    scheduler = core(activation_mode='always')
    scheduler.playback_started(0.0)
    scheduler.cancel()
    assert scheduler.due(100.0) is PlaybackDecision.NONE


def test_shutdown_prevents_future_activation():
    scheduler = core()
    scheduler.shutdown()
    scheduler.set_drive_mode(5, 1.0)
    assert scheduler.due(1.0) is PlaybackDecision.NONE


@pytest.mark.parametrize('interval', [0.0, -1.0, float('inf'), float('nan')])
def test_invalid_interval_is_rejected(interval):
    with pytest.raises(ValueError, match='interval_sec'):
        core(interval_sec=interval)


def test_activation_mode_is_validated():
    with pytest.raises(ValueError, match='activation_mode'):
        core(activation_mode='invalid')


def test_always_mode_is_immediately_due():
    assert core(activation_mode='always').due(0.0) is PlaybackDecision.PLAY


def test_shutdown_helper_terminates_child_process():
    class Process:
        def __init__(self):
            self.terminated = False

        def poll(self):
            return None

        def terminate(self):
            self.terminated = True

        def wait(self, timeout):
            assert timeout == 1.0

    process = Process()
    stop_child_process(process)
    assert process.terminated is True
