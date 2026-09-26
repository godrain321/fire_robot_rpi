"""Mode 3: play the fire-guide greeting while rotating in place once."""

from __future__ import annotations

import math
import shutil
import subprocess
import time
from pathlib import Path
from typing import Optional

from ament_index_python.packages import get_package_share_directory
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Empty, Int32, String

from .mode3_audio_guide import (
    build_amixer_command,
    build_aplay_command,
    discover_usb_alsa_device,
    expand_audio_directory,
    inaccessible_alsa_control_path,
)


INTRO_TEXT = (
    '안녕하세요 저는 화재대피안내로봇입니다 '
    '안전한출구로 안내해드리겠습니다'
)


class TimedOneTurn:
    """Time a single commanded rotation without accumulating timer drift."""

    def __init__(self, angular_speed_radps: float):
        speed = abs(float(angular_speed_radps))
        if not math.isfinite(speed) or speed <= 0.0:
            raise ValueError('angular_speed_radps must be positive and finite')
        self.angular_speed = speed
        self.duration_sec = 2.0 * math.pi / speed
        self.started_at: Optional[float] = None

    @property
    def active(self) -> bool:
        return self.started_at is not None

    def start(self, now: float) -> None:
        self.started_at = float(now)

    def cancel(self) -> None:
        self.started_at = None

    def command(self, now: float) -> tuple[float, bool]:
        if self.started_at is None:
            return 0.0, True
        if float(now) - self.started_at >= self.duration_sec:
            self.started_at = None
            return 0.0, True
        return self.angular_speed, False


class Mode3GreetingSpin(Node):
    def __init__(self) -> None:
        super().__init__('mode3_greeting_spin')
        defaults = {
            'angular_speed_radps': 1.0,
            'publish_rate_hz': 20.0,
            'audio_directory': '~/fire_robot_audio',
            'audio_file': 'mode3_intro.wav',
            'audio_device': 'auto',
            'player_executable': 'aplay',
            'playback_volume_percent': 100,
        }
        for name, default in defaults.items():
            self.declare_parameter(name, default)
        value = lambda name: self.get_parameter(name).value
        publish_rate = float(value('publish_rate_hz'))
        if publish_rate <= 0.0:
            raise ValueError('publish_rate_hz must be positive')
        self.turn = TimedOneTurn(float(value('angular_speed_radps')))
        self.audio_file_name = str(value('audio_file')).strip()
        if (not self.audio_file_name
                or Path(self.audio_file_name).name != self.audio_file_name):
            raise ValueError('audio_file must be a plain file name')
        preferred = expand_audio_directory(str(value('audio_directory')))
        packaged = Path(get_package_share_directory('inno_robot_bringup')) / 'audio'
        preferred_file = preferred / self.audio_file_name
        self.audio_file = (
            preferred_file if preferred_file.is_file()
            else packaged / self.audio_file_name
        )
        self.configured_device = str(value('audio_device')).strip()
        self.player_executable = str(value('player_executable')).strip()
        self.playback_volume_percent = int(value('playback_volume_percent'))
        if (not self.configured_device or not self.player_executable
                or not 0 <= self.playback_volume_percent <= 100):
            raise ValueError('Mode 3 audio parameters are invalid')

        latched = QoSProfile(depth=1)
        latched.reliability = ReliabilityPolicy.RELIABLE
        latched.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.velocity_publisher = self.create_publisher(
            Twist, '/cmd_vel_auto', 10
        )
        self.status_publisher = self.create_publisher(
            String, '/mode3_demo/status', latched
        )
        self.create_subscription(
            Empty, '/mode3/demo_request', self._on_request, 10
        )
        self.create_subscription(
            Empty, '/autonomy_cancel', self._on_cancel, 10
        )
        self.create_subscription(
            Int32, '/operator_mode', self._on_operator_mode, latched
        )
        self.create_timer(1.0 / publish_rate, self._tick)
        self.operator_mode = 1
        self.player_process: Optional[subprocess.Popen] = None
        self._reported_complete = True
        self._publish_status('READY')

    def _publish_status(self, status: str) -> None:
        self.status_publisher.publish(String(data=status))

    def _resolve_device(self) -> Optional[str]:
        if self.configured_device.lower() != 'auto':
            return self.configured_device
        return discover_usb_alsa_device()

    def _configure_volume(self, device: str) -> None:
        executable = shutil.which('amixer')
        if executable is None:
            return
        command = build_amixer_command(
            executable, device, self.playback_volume_percent
        )
        if command is None:
            return
        subprocess.run(
            command,
            stdin=subprocess.DEVNULL,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            check=False,
        )

    def _play_intro(self) -> None:
        if not self.audio_file.is_file():
            self._publish_status('ERROR:AUDIO_FILE_NOT_FOUND')
            self.get_logger().error(
                f'[MODE 3] 안내 음원 없음: {self.audio_file}'
            )
            return
        executable = shutil.which(self.player_executable)
        device = self._resolve_device()
        if executable is None:
            self._publish_status('ERROR:PLAYER_NOT_FOUND')
            self.get_logger().error('[MODE 3] aplay 실행 파일을 찾지 못했습니다.')
            return
        if device is None:
            self._publish_status('ERROR:USB_AUDIO_NOT_FOUND')
            self.get_logger().error('[MODE 3] USB 오디오 장치를 찾지 못했습니다.')
            return
        if inaccessible_alsa_control_path(device) is not None:
            self._publish_status('ERROR:AUDIO_DEVICE_PERMISSION')
            self.get_logger().error('[MODE 3] USB 오디오 장치 접근 권한이 없습니다.')
            return
        self._configure_volume(device)
        if self.player_process is not None and self.player_process.poll() is None:
            self.player_process.terminate()
        try:
            self.player_process = subprocess.Popen(
                build_aplay_command(executable, device, self.audio_file),
                stdin=subprocess.DEVNULL,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.PIPE,
                text=True,
            )
        except OSError as error:
            self._publish_status(f'ERROR:PLAYBACK:{type(error).__name__}')
            self.get_logger().error(f'[MODE 3] 음성 재생 실패: {error}')
            return
        self.get_logger().warning(f'[MODE 3 음성] {INTRO_TEXT}')

    def _on_request(self, _message: Empty) -> None:
        if self.operator_mode != 3:
            self.get_logger().warning(
                '[MODE 3] operator_mode=3 확인 전 요청 수신; 동작을 시작합니다.'
            )
        self.velocity_publisher.publish(Twist())
        self.turn.start(time.monotonic())
        self._reported_complete = False
        self._play_intro()
        self._publish_status('RUNNING:VOICE_AND_ONE_TURN')
        self.get_logger().warning(
            f'[MODE 3] 제자리 1회전 시작: '
            f'{self.turn.angular_speed:.2f} rad/s, '
            f'{self.turn.duration_sec:.2f} s'
        )

    def _on_cancel(self, _message: Empty) -> None:
        if self.turn.active:
            self.turn.cancel()
            self._reported_complete = True
            self.velocity_publisher.publish(Twist())
            self._publish_status('CANCELLED')

    def _on_operator_mode(self, message: Int32) -> None:
        self.operator_mode = int(message.data)
        if self.operator_mode != 3 and self.turn.active:
            self._on_cancel(Empty())

    def _tick(self) -> None:
        angular, finished = self.turn.command(time.monotonic())
        command = Twist()
        command.angular.z = angular
        self.velocity_publisher.publish(command)
        if finished and angular == 0.0 and self.turn.started_at is None:
            # Only publish COMPLETE once after a real run.
            if not self._reported_complete:
                self._reported_complete = True
                self._publish_status('COMPLETE:ONE_TURN')
                self.get_logger().info('[MODE 3] 제자리 1회전 완료, 모터 정지')
        else:
            self._reported_complete = False

    def destroy_node(self):
        if rclpy.ok():
            self.velocity_publisher.publish(Twist())
        if self.player_process is not None and self.player_process.poll() is None:
            self.player_process.terminate()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = Mode3GreetingSpin()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
