"""Offline, non-blocking periodic evacuation voice ROS 2 node."""

from __future__ import annotations

import os
from pathlib import Path
import re
import shutil
import subprocess
import time
from typing import Optional

from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Empty, Int32, String

from .periodic_voice_core import (
    PlaybackDecision, PeriodicVoiceCore, stop_child_process,
)


def expand_audio_directory(value: str) -> Path:
    return Path(os.path.expandvars(os.path.expanduser(str(value))))


def select_audio_file(preferred_directory: Path, packaged_directory: Path, name: str) -> Path:
    preferred = preferred_directory / name
    return preferred if preferred.is_file() else packaged_directory / name


def discover_usb_alsa_device(cards_text: Optional[str] = None) -> Optional[str]:
    if cards_text is None:
        try:
            cards_text = Path('/proc/asound/cards').read_text(
                encoding='utf-8', errors='replace'
            )
        except OSError:
            return None
    entries = re.split(r'(?=^\s*\d+\s+\[[^\]]+\]\s*:)', cards_text, flags=re.M)
    for entry in entries:
        header = re.match(r'^\s*\d+\s+\[([^\]]+)\]\s*:', entry)
        if header is not None and 'usb' in entry.lower():
            card = header.group(1).strip()
            if card:
                return f'plughw:CARD={card},DEV=0'
    return None


def alsa_card_value(device: str) -> Optional[str]:
    match = re.search(r'(?:^|[:,])CARD=([^,]+)', str(device))
    return None if match is None else (match.group(1).strip() or None)


def alsa_control_path(device: str, cards_text: Optional[str] = None) -> Optional[Path]:
    card = alsa_card_value(device)
    if card is None:
        return None
    if card.isdigit():
        return Path(f'/dev/snd/controlC{card}')
    if cards_text is None:
        try:
            cards_text = Path('/proc/asound/cards').read_text(
                encoding='utf-8', errors='replace'
            )
        except OSError:
            return None
    match = re.search(rf'^\s*(\d+)\s+\[{re.escape(card)}\s*\]\s*:', cards_text, re.M)
    return None if match is None else Path(f'/dev/snd/controlC{match.group(1)}')


def inaccessible_alsa_control_path(device: str) -> Optional[Path]:
    path = alsa_control_path(device)
    if path is None or not path.exists() or os.access(path, os.R_OK | os.W_OK):
        return None
    return path


def build_aplay_command(executable: str, device: str, audio_file: Path) -> list[str]:
    return [executable, '--quiet', '-D', device, str(audio_file)]


def build_amixer_command(
    executable: str, device: str, volume_percent: int
) -> Optional[list[str]]:
    card = alsa_card_value(device)
    if card is None:
        return None
    return [
        executable, '--quiet', '--card', card, 'sset', 'Speaker',
        f'{volume_percent}%', 'unmute',
    ]


class PeriodicEvacuationVoiceNode(Node):
    def __init__(self, monotonic_clock=time.monotonic) -> None:
        super().__init__('periodic_evacuation_voice_node')
        defaults = {
            'enabled': True,
            'interval_sec': 7.0,
            'play_immediately': True,
            'active_drive_mode': 5,
            'audio_directory': '~/fire_robot_audio',
            'audio_file': 'evacuation_guide.wav',
            'audio_device': 'auto',
            'player_executable': 'aplay',
            'playback_volume_percent': 100,
            'activation_mode': 'drive_mode',
        }
        for name, default in defaults.items():
            self.declare_parameter(name, default)
        value = lambda name: self.get_parameter(name).value
        self.enabled = bool(value('enabled'))
        self.clock = monotonic_clock
        self.configured_device = str(value('audio_device')).strip()
        self.player_executable = str(value('player_executable')).strip()
        self.volume_percent = int(value('playback_volume_percent'))
        audio_name = str(value('audio_file')).strip()
        if (
            not audio_name or Path(audio_name).name != audio_name
            or not self.configured_device or not self.player_executable
            or not 0 <= self.volume_percent <= 100
        ):
            raise ValueError('periodic evacuation voice parameters are invalid')
        packaged = Path(get_package_share_directory('inno_evacuation_voice')) / 'audio'
        self.audio_file = select_audio_file(
            expand_audio_directory(str(value('audio_directory'))), packaged, audio_name
        )
        self.core = PeriodicVoiceCore(
            float(value('interval_sec')),
            play_immediately=bool(value('play_immediately')),
            active_drive_mode=int(value('active_drive_mode')),
            activation_mode=str(value('activation_mode')).strip(),
            now=self.clock(),
        )
        transient = QoSProfile(depth=1)
        transient.reliability = ReliabilityPolicy.RELIABLE
        transient.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.status_publisher = self.create_publisher(
            String, '/evacuation_voice/status', transient
        )
        self.create_subscription(Int32, '/drive_mode', self._on_drive_mode, 10)
        self.create_subscription(Empty, '/autonomy_cancel', self._on_cancel, 10)
        self.create_timer(0.1, self._tick)
        self.player_process: Optional[subprocess.Popen] = None
        self.last_status = ''
        self._publish_state('READY' if self.enabled else 'DISABLED')

    def _publish_state(self, state: str, *, error: bool = False) -> None:
        self.status_publisher.publish(String(data=state))
        if state == self.last_status:
            return
        self.last_status = state
        log = self.get_logger().error if error else self.get_logger().info
        log(f'[ROBOT] [주기 대피 음성] {state}')

    def _on_drive_mode(self, message: Int32) -> None:
        if not self.enabled:
            self._publish_state('DISABLED')
            return
        was_stopped = self.core.set_drive_mode(int(message.data), self.clock())
        if was_stopped:
            self._stop_player()
            self._publish_state('STOPPED:DRIVE_MODE_CHANGED')
        elif not self.core.active:
            self._publish_state('INACTIVE:DRIVE_MODE')

    def _on_cancel(self, _message: Empty) -> None:
        self.core.cancel()
        self._stop_player()
        self._publish_state(
            'STOPPED:AUTONOMY_CANCEL' if self.enabled else 'DISABLED'
        )

    def _resolve_device(self) -> Optional[str]:
        if self.configured_device.lower() != 'auto':
            return self.configured_device
        return discover_usb_alsa_device()

    def _tick(self) -> None:
        self._poll_player()
        if not self.enabled:
            return
        now = self.clock()
        decision = self.core.due(now)
        if decision is PlaybackDecision.BUSY:
            self._publish_state('BUSY:ALREADY_PLAYING')
        elif decision is PlaybackDecision.PLAY:
            self._start_playback(now)

    def _start_playback(self, now: float) -> None:
        error = None
        if not self.audio_file.is_file():
            error = 'ERROR:AUDIO_FILE_NOT_FOUND'
        executable = shutil.which(self.player_executable)
        if error is None and executable is None:
            error = 'ERROR:PLAYER_NOT_FOUND'
        device = self._resolve_device() if error is None else None
        if error is None and device is None:
            error = 'ERROR:USB_AUDIO_NOT_FOUND'
        if error is None and inaccessible_alsa_control_path(device) is not None:
            error = 'ERROR:AUDIO_DEVICE_PERMISSION'
        if error is not None:
            self.core.playback_start_failed(now)
            self._publish_state(error, error=True)
            return
        self._configure_volume(device)
        command = build_aplay_command(executable, device, self.audio_file)
        try:
            self.player_process = subprocess.Popen(
                command, stdin=subprocess.DEVNULL, stdout=subprocess.DEVNULL,
                stderr=subprocess.PIPE, text=True, shell=False,
            )
        except (OSError, PermissionError) as exc:
            self.core.playback_start_failed(now)
            state = (
                'ERROR:AUDIO_DEVICE_PERMISSION'
                if isinstance(exc, PermissionError) else 'ERROR:PLAYBACK_START'
            )
            self._publish_state(state, error=True)
            return
        self.core.playback_started(now)
        self._publish_state('PLAYING:EVACUATION_GUIDE')

    def _configure_volume(self, device: str) -> None:
        executable = shutil.which('amixer')
        command = None if executable is None else build_amixer_command(
            executable, device, self.volume_percent
        )
        if command is None:
            return
        try:
            subprocess.run(
                command, stdin=subprocess.DEVNULL, stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL, timeout=2.0, check=False, shell=False,
            )
        except (OSError, subprocess.TimeoutExpired):
            self.get_logger().warning('[ROBOT] [주기 대피 음성] amixer 설정 실패')

    def _poll_player(self) -> None:
        process = self.player_process
        if process is None or process.poll() is None:
            return
        _, stderr = process.communicate()
        code = int(process.returncode)
        self.player_process = None
        self.core.playback_finished()
        if code == 0:
            self._publish_state('COMPLETED:EVACUATION_GUIDE')
        else:
            self._publish_state(f'ERROR:PLAYBACK_EXIT:{code}', error=True)
            detail = (stderr or '').strip().replace('\n', ' ')
            if detail:
                self.get_logger().error(detail)

    def _stop_player(self) -> None:
        process = self.player_process
        self.player_process = None
        self.core.playback_finished()
        stop_child_process(process)

    def destroy_node(self):
        self.core.shutdown()
        self._stop_player()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = PeriodicEvacuationVoiceNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except ValueError as exc:
        print(f'periodic_evacuation_voice_node: {exc}')
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
