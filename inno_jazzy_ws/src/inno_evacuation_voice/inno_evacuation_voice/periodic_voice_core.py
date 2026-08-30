"""ROS-independent monotonic scheduler for periodic evacuation speech."""

from __future__ import annotations

from enum import Enum
import math
import subprocess


class PlaybackDecision(Enum):
    NONE = 'none'
    PLAY = 'play'
    BUSY = 'busy'


class PeriodicVoiceCore:
    def __init__(
        self,
        interval_sec: float = 7.0,
        *,
        play_immediately: bool = True,
        active_drive_mode: int = 5,
        activation_mode: str = 'drive_mode',
        now: float = 0.0,
    ) -> None:
        if not math.isfinite(interval_sec) or interval_sec <= 0.0:
            raise ValueError('interval_sec must be finite and greater than zero')
        if activation_mode not in {'drive_mode', 'always'}:
            raise ValueError("activation_mode must be 'drive_mode' or 'always'")
        self.interval_sec = float(interval_sec)
        self.play_immediately = bool(play_immediately)
        self.active_drive_mode = int(active_drive_mode)
        self.activation_mode = activation_mode
        self.active = activation_mode == 'always'
        self.playing = False
        self.stopped = False
        self.next_due = (
            float(now) if self.active and self.play_immediately
            else float(now) + self.interval_sec
        )

    def set_drive_mode(self, mode: int, now: float) -> bool:
        """Apply a drive-mode transition; return whether activity stopped."""
        if self.activation_mode != 'drive_mode' or self.stopped:
            return False
        should_activate = int(mode) == self.active_drive_mode
        if should_activate and not self.active:
            self.active = True
            self.next_due = (
                float(now) if self.play_immediately
                else float(now) + self.interval_sec
            )
            return False
        if not should_activate and self.active:
            self.active = False
            self.playing = False
            return True
        return False

    def due(self, now: float) -> PlaybackDecision:
        if self.stopped or not self.active or float(now) < self.next_due:
            return PlaybackDecision.NONE
        if self.playing:
            self.next_due = float(now) + self.interval_sec
            return PlaybackDecision.BUSY
        return PlaybackDecision.PLAY

    def playback_started(self, now: float) -> None:
        self.playing = True
        self.next_due = float(now) + self.interval_sec

    def playback_start_failed(self, now: float) -> None:
        self.playing = False
        self.next_due = float(now) + self.interval_sec

    def playback_finished(self) -> None:
        self.playing = False

    def cancel(self) -> None:
        self.active = False
        self.playing = False

    def shutdown(self) -> None:
        self.cancel()
        self.stopped = True


def stop_child_process(process, timeout_sec: float = 1.0) -> None:
    """Terminate one active player, escalating to kill after a short timeout."""
    if process is None or process.poll() is not None:
        return
    process.terminate()
    try:
        process.wait(timeout=timeout_sec)
    except subprocess.TimeoutExpired:
        process.kill()
        process.wait(timeout=timeout_sec)
