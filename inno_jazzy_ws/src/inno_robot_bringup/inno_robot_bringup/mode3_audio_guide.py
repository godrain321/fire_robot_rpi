"""Play one evacuation guide when Mode 3 or 4 marks a survivor blue."""

from __future__ import annotations

import os
import re
import shutil
import subprocess
from pathlib import Path
from typing import Optional

from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Int32, String


MODE3_SURVIVOR_CONFIRMED_STATUS = 'MODE3_PERSON_CONFIRMED:MARKER_BLUE'
MODE4_SURVIVOR_CONFIRMED_STATUS = 'MODE4_SURVIVOR_CONFIRMED:MARKER_BLUE'
# Preserve the original public constant for callers/tests using the Mode 3
# integration name.
SURVIVOR_CONFIRMED_STATUS = MODE3_SURVIVOR_CONFIRMED_STATUS
DEFAULT_AUDIO_FILES = (
    'survivor_detected.wav',
    'follow_me.wav',
    'arrived_exit.wav',
    'emergency_stop.wav',
)


def expand_audio_directory(value: str) -> Path:
    """Expand shell-style home and environment variables without a shell."""
    return Path(os.path.expandvars(os.path.expanduser(str(value))))


def select_audio_directory(preferred: Path, packaged: Path) -> Path:
    """Prefer the operator directory, then use packaged WAV assets."""
    if all((preferred / name).is_file() for name in DEFAULT_AUDIO_FILES):
        return preferred
    if all((packaged / name).is_file() for name in DEFAULT_AUDIO_FILES):
        return packaged
    return preferred


def discover_usb_alsa_device(
    cards_text: Optional[str] = None,
) -> Optional[str]:
    """Return an ALSA plughw device for the first USB audio card."""
    if cards_text is None:
        try:
            cards_text = Path('/proc/asound/cards').read_text(
                encoding='utf-8', errors='replace'
            )
        except OSError:
            return None

    entries = re.split(r'(?=^\s*\d+\s+\[[^\]]+\]\s*:)', cards_text, flags=re.M)
    for entry in entries:
        header = re.match(
            r'^\s*\d+\s+\[([^\]]+)\]\s*:\s*([^\n]*)',
            entry,
        )
        if header is None or 'usb' not in entry.lower():
            continue
        card_id = header.group(1).strip()
        if card_id:
            return f'plughw:CARD={card_id},DEV=0'
    return None


def alsa_card_value(device: str) -> Optional[str]:
    """Extract the ALSA CARD value from a hw/plughw device string."""
    card_match = re.search(r'(?:^|[:,])CARD=([^,]+)', str(device))
    if card_match is None:
        return None
    value = card_match.group(1).strip()
    return value or None


def alsa_control_path(
    device: str,
    cards_text: Optional[str] = None,
) -> Optional[Path]:
    """Map an explicit ALSA CARD value to its control device path."""
    card_value = alsa_card_value(device)
    if card_value is None:
        return None
    if card_value.isdigit():
        return Path(f'/dev/snd/controlC{card_value}')
    if cards_text is None:
        try:
            cards_text = Path('/proc/asound/cards').read_text(
                encoding='utf-8', errors='replace'
            )
        except OSError:
            return None
    pattern = re.compile(
        rf'^\s*(\d+)\s+\[{re.escape(card_value)}\s*\]\s*:',
        flags=re.M,
    )
    match = pattern.search(cards_text)
    if match is None:
        return None
    return Path(f'/dev/snd/controlC{match.group(1)}')


def inaccessible_alsa_control_path(device: str) -> Optional[Path]:
    """Return the existing ALSA control path when this user cannot open it."""
    path = alsa_control_path(device)
    if path is None or not path.exists():
        return None
    if os.access(path, os.R_OK | os.W_OK):
        return None
    return path


def build_aplay_command(
    executable: str, device: str, audio_file: Path
) -> list[str]:
    """Build a shell-free WAV playback command."""
    return [executable, '--quiet', '-D', device, str(audio_file)]


def build_amixer_command(
    executable: str,
    device: str,
    volume_percent: int,
) -> Optional[list[str]]:
    """Build the USB-card Speaker volume command when CARD is explicit."""
    card = alsa_card_value(device)
    if card is None:
        return None
    return [
        executable,
        '--quiet',
        '--card',
        card,
        'sset',
        'Speaker',
        f'{volume_percent}%',
        'unmute',
    ]


class Mode3AudioGuide(Node):
    """React once to each live Mode 3/4 survivor-confirmed transition."""

    def __init__(self) -> None:
        """Configure the trigger, WAV directory, and non-blocking player."""
        super().__init__('mode3_audio_guide')
        self.declare_parameter('enabled', True)
        self.declare_parameter('audio_directory', '~/fire_robot_audio')
        self.declare_parameter('survivor_audio_file', 'survivor_detected.wav')
        self.declare_parameter('audio_device', 'auto')
        self.declare_parameter('player_executable', 'aplay')
        self.declare_parameter('playback_volume_percent', 100)

        self.enabled = bool(self.get_parameter('enabled').value)
        preferred_audio_directory = expand_audio_directory(
            str(self.get_parameter('audio_directory').value)
        )
        packaged_audio_directory = Path(
            get_package_share_directory('inno_robot_bringup')
        ) / 'audio'
        self.audio_directory = select_audio_directory(
            preferred_audio_directory, packaged_audio_directory
        )
        self.survivor_audio_file = str(
            self.get_parameter('survivor_audio_file').value
        ).strip()
        self.configured_device = str(
            self.get_parameter('audio_device').value
        ).strip()
        self.player_executable = str(
            self.get_parameter('player_executable').value
        ).strip()
        self.playback_volume_percent = int(
            self.get_parameter('playback_volume_percent').value
        )
        if (
            not self.survivor_audio_file
            or Path(self.survivor_audio_file).name != self.survivor_audio_file
            or not self.configured_device
            or not self.player_executable
            or not 0 <= self.playback_volume_percent <= 100
        ):
            raise ValueError('Mode 3 audio parameters are invalid')

        transient = QoSProfile(depth=1)
        transient.reliability = ReliabilityPolicy.RELIABLE
        transient.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.status_publisher = self.create_publisher(
            String, '/mode3_audio_status', transient
        )
        self.create_subscription(Int32, '/drive_mode', self._on_drive_mode, 10)
        self.create_subscription(
            String, '/mode3_status', self._on_mode3_status, transient
        )
        self.create_subscription(
            String, '/mode4_status', self._on_mode4_status, transient
        )
        self.create_timer(0.2, self._poll_player)

        self.drive_mode = 1
        self.last_mode3_status = ''
        self.last_mode4_status = ''
        # Do not replay a transient-local survivor status that was published
        # before this node started.  A live non-trigger state arms the node.
        self.armed_for_mode3 = False
        self.armed_for_mode4 = False
        self.player_process: Optional[subprocess.Popen] = None

        missing = [
            name for name in DEFAULT_AUDIO_FILES
            if not (self.audio_directory / name).is_file()
        ]
        if missing:
            self._publish_state('AUDIO_FILES_MISSING:' + ','.join(missing))
            self.get_logger().error(
                '[ROBOT] [음성 오류] WAV 파일 없음: ' + ', '.join(missing)
            )
        elif not self.enabled:
            self._publish_state('DISABLED')
            self.get_logger().warning('[ROBOT] [음성] 모드3 음성 안내 비활성화')
        else:
            self._publish_state('READY')
            device = self._resolve_device()
            if device is None:
                self.get_logger().warning(
                    '[ROBOT] [음성] USB 사운드카드를 아직 찾지 못했습니다. '
                    '요구조자 확정 시 다시 확인합니다.'
                )
            elif inaccessible_alsa_control_path(device) is not None:
                self._report_permission_error(device)
            else:
                self._configure_playback_volume(device)
                self.get_logger().info(
                    f'[ROBOT] [음성] 모드3/4 안내 준비 완료: {device}'
                )

    def _publish_state(self, state: str) -> None:
        self.status_publisher.publish(String(data=state))

    def _on_drive_mode(self, message: Int32) -> None:
        self.drive_mode = int(message.data)

    def _on_mode3_status(self, message: String) -> None:
        status = message.data.strip().upper()
        self.last_mode3_status, self.armed_for_mode3 = (
            self._handle_survivor_status(
                status=status,
                trigger_status=MODE3_SURVIVOR_CONFIRMED_STATUS,
                expected_drive_mode=3,
                previous_status=self.last_mode3_status,
                armed=self.armed_for_mode3,
            )
        )

    def _on_mode4_status(self, message: String) -> None:
        status = message.data.strip().upper()
        self.last_mode4_status, self.armed_for_mode4 = (
            self._handle_survivor_status(
                status=status,
                trigger_status=MODE4_SURVIVOR_CONFIRMED_STATUS,
                expected_drive_mode=4,
                previous_status=self.last_mode4_status,
                armed=self.armed_for_mode4,
            )
        )

    def _handle_survivor_status(
        self,
        *,
        status: str,
        trigger_status: str,
        expected_drive_mode: int,
        previous_status: str,
        armed: bool,
    ) -> tuple[str, bool]:
        if status != trigger_status:
            return status, True
        if previous_status == trigger_status:
            return status, armed
        if not armed:
            self.get_logger().debug(
                'Ignored survivor status published before audio-node startup.'
            )
            return status, armed
        armed = False
        if self.drive_mode != expected_drive_mode or not self.enabled:
            return status, armed
        self._play_survivor_guide()
        return status, armed

    def _resolve_device(self) -> Optional[str]:
        if self.configured_device.lower() != 'auto':
            return self.configured_device
        return discover_usb_alsa_device()

    def _play_survivor_guide(self) -> None:
        audio_file = self.audio_directory / self.survivor_audio_file
        if not audio_file.is_file():
            self._publish_state('ERROR:AUDIO_FILE_NOT_FOUND')
            self.get_logger().error(
                f'[ROBOT] [음성 오류] 파일을 찾을 수 없습니다: {audio_file}'
            )
            return
        executable = shutil.which(self.player_executable)
        if executable is None:
            self._publish_state('ERROR:PLAYER_NOT_FOUND')
            self.get_logger().error(
                f'[ROBOT] [음성 오류] {self.player_executable} 실행 파일 없음'
            )
            return
        device = self._resolve_device()
        if device is None:
            self._publish_state('ERROR:USB_AUDIO_NOT_FOUND')
            self.get_logger().error(
                '[ROBOT] [음성 오류] USB 사운드카드가 연결되지 않았습니다.'
            )
            return
        if inaccessible_alsa_control_path(device) is not None:
            self._report_permission_error(device)
            return
        self._configure_playback_volume(device)
        if (
            self.player_process is not None
            and self.player_process.poll() is None
        ):
            self._publish_state('BUSY:ALREADY_PLAYING')
            self.get_logger().warning(
                '[ROBOT] [음성] 이전 안내가 재생 중이므로 중복 재생하지 않습니다.'
            )
            return
        command = build_aplay_command(executable, device, audio_file)
        try:
            self.player_process = subprocess.Popen(
                command,
                stdin=subprocess.DEVNULL,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.PIPE,
                text=True,
            )
        except OSError as error:
            self._publish_state(f'ERROR:PLAYBACK_START:{type(error).__name__}')
            self.get_logger().error(f'[ROBOT] [음성 오류] 재생 시작 실패: {error}')
            return
        self._publish_state('PLAYING:SURVIVOR_DETECTED')
        self.get_logger().info(
            '[ROBOT] [음성 재생] 요구조자 발견 안내를 한 번 재생합니다.'
        )

    def _report_permission_error(self, device: str) -> None:
        self._publish_state('ERROR:AUDIO_DEVICE_PERMISSION')
        path = inaccessible_alsa_control_path(device)
        self.get_logger().error(
            f'[ROBOT] [음성 오류] {path or device} 접근 권한이 없습니다. '
            'seeno04 사용자를 audio 그룹에 추가하고 실행을 다시 시작하세요.'
        )

    def _configure_playback_volume(self, device: str) -> None:
        executable = shutil.which('amixer')
        if executable is None:
            self.get_logger().warning(
                '[ROBOT] [음성] amixer가 없어 USB 출력 음량을 설정하지 못했습니다.'
            )
            return
        command = build_amixer_command(
            executable, device, self.playback_volume_percent
        )
        if command is None:
            return
        try:
            result = subprocess.run(
                command,
                stdin=subprocess.DEVNULL,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.PIPE,
                text=True,
                timeout=2.0,
                check=False,
            )
        except (OSError, subprocess.TimeoutExpired) as error:
            self.get_logger().warning(
                f'[ROBOT] [음성] USB 출력 음량 자동 설정 실패: {error}'
            )
            return
        if result.returncode == 0:
            self.get_logger().info(
                '[ROBOT] [음성] USB 스피커 출력 음량: '
                f'{self.playback_volume_percent}%'
            )
            return
        detail = (result.stderr or '').strip().replace('\n', ' ')
        self.get_logger().warning(
            '[ROBOT] [음성] USB 출력 음량 자동 설정 실패: ' + detail
        )

    def _poll_player(self) -> None:
        process = self.player_process
        if process is None or process.poll() is None:
            return
        _, stderr = process.communicate()
        return_code = process.returncode
        self.player_process = None
        if return_code == 0:
            self._publish_state('COMPLETED:SURVIVOR_DETECTED')
            self.get_logger().info('[ROBOT] [음성] 요구조자 안내 재생 완료')
            return
        detail = (stderr or '').strip().replace('\n', ' ')
        self._publish_state(f'ERROR:PLAYBACK_EXIT:{return_code}')
        self.get_logger().error(
            f'[ROBOT] [음성 오류] 재생 실패({return_code}): {detail}'
        )

    def destroy_node(self):
        """Stop an active speaker process before destroying the ROS node."""
        process = self.player_process
        if process is not None and process.poll() is None:
            process.terminate()
            try:
                process.wait(timeout=1.0)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait(timeout=1.0)
        return super().destroy_node()


def main(args=None) -> None:
    """Run the Mode 3 audio guide node."""
    rclpy.init(args=args)
    node = None
    try:
        node = Mode3AudioGuide()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except ValueError as error:
        if node is None:
            print(f'mode3_audio_guide: {error}')
        else:
            node.get_logger().error(str(error))
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
