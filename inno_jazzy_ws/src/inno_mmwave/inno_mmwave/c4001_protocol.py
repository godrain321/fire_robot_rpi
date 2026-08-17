"""Pure helpers for the DFRobot C4001 ASCII UART protocol.

The sensor can prepend its ``DFRobot:/>`` prompt, return frames in arbitrary
UART chunks, and place several frames in one read.  ``C4001StreamParser``
therefore owns a bounded byte buffer and only emits complete ``$DFDMD...*``
frames.  No ROS or serial dependency is used here so the parser can be tested
without hardware.
"""

from dataclasses import dataclass
import math
from typing import List, Optional, Union


DFDMD_MARKER = b'$DFDMD,'
FRAME_END = ord('*')
MAX_PROTOCOL_DISTANCE_M = 25.0
MAX_PROTOCOL_SPEED_MPS = 10.0
MAX_ENERGY = (1 << 32) - 1


class C4001FrameError(ValueError):
    """Raised when a complete C4001 frame violates the documented format."""


@dataclass(frozen=True)
class C4001Measurement:
    """One distance/speed-mode sample reported by the C4001."""

    target_count: int
    target_id: Optional[int]
    distance_m: Optional[float]
    speed_mps: Optional[float]
    energy: Optional[int]
    raw_frame: str

    @property
    def detected(self) -> bool:
        return self.target_count == 1


def _decode_ascii(frame: Union[str, bytes, bytearray, memoryview]) -> str:
    if isinstance(frame, str):
        return frame
    if not isinstance(frame, (bytes, bytearray, memoryview)):
        raise TypeError('frame must be str or bytes-like')
    try:
        return bytes(frame).decode('ascii')
    except UnicodeDecodeError as error:
        raise C4001FrameError('frame is not ASCII') from error


def _finite_float(value: str, field: str) -> float:
    try:
        parsed = float(value.strip())
    except ValueError as error:
        raise C4001FrameError(f'invalid {field}') from error
    if not math.isfinite(parsed):
        raise C4001FrameError(f'non-finite {field}')
    return parsed


def parse_dfdmd_frame(
    frame: Union[str, bytes, bytearray, memoryview],
) -> C4001Measurement:
    """Parse one documented ``$DFDMD`` active-report frame.

    The seven parameters are target count, target ID, distance in metres,
    signed radial speed in metres/second, energy, and two reserved fields.
    A prompt before ``$DFDMD`` is tolerated because real modules emit it.
    """

    text = _decode_ascii(frame).strip()
    marker = DFDMD_MARKER.decode('ascii')
    start = text.find(marker)
    if start < 0:
        raise C4001FrameError('missing $DFDMD marker')
    end = text.find('*', start + len(marker))
    if end < 0:
        raise C4001FrameError('incomplete frame')
    if text[end + 1 :].strip():
        raise C4001FrameError('unexpected data after frame terminator')

    raw_frame = text[start : end + 1]
    fields = text[start + len(marker) : end].split(',')
    if len(fields) != 7:
        raise C4001FrameError(
            f'$DFDMD requires 7 parameters, received {len(fields)}'
        )

    try:
        target_count = int(fields[0].strip())
    except ValueError as error:
        raise C4001FrameError('invalid target count') from error
    if target_count not in (0, 1):
        raise C4001FrameError('target count must be 0 or 1')

    if target_count == 0:
        return C4001Measurement(
            target_count=0,
            target_id=None,
            distance_m=None,
            speed_mps=None,
            energy=None,
            raw_frame=raw_frame,
        )

    try:
        target_id = int(fields[1].strip())
    except ValueError as error:
        raise C4001FrameError('invalid target ID') from error
    if target_id != 1:
        raise C4001FrameError('single-target C4001 must report target ID 1')

    distance_m = _finite_float(fields[2], 'distance')
    if not 0.0 <= distance_m <= MAX_PROTOCOL_DISTANCE_M:
        raise C4001FrameError('distance is outside the protocol range')

    speed_mps = _finite_float(fields[3], 'speed')
    if abs(speed_mps) > MAX_PROTOCOL_SPEED_MPS:
        raise C4001FrameError('speed is outside the protocol range')

    energy_value = _finite_float(fields[4], 'energy')
    if (
        energy_value < 0.0
        or energy_value > MAX_ENERGY
        or not energy_value.is_integer()
    ):
        raise C4001FrameError('energy must be an unsigned 32-bit integer')

    return C4001Measurement(
        target_count=target_count,
        target_id=target_id,
        distance_m=distance_m,
        speed_mps=speed_mps,
        energy=int(energy_value),
        raw_frame=raw_frame,
    )


class C4001StreamParser:
    """Incrementally extract valid ``$DFDMD`` frames from noisy UART bytes."""

    def __init__(self, max_frame_bytes: int = 256) -> None:
        if max_frame_bytes < len(DFDMD_MARKER) + 2:
            raise ValueError('max_frame_bytes is too small')
        self.max_frame_bytes = int(max_frame_bytes)
        self._buffer = bytearray()
        self.malformed_frames = 0
        self.discarded_bytes = 0

    @property
    def buffered_bytes(self) -> int:
        return len(self._buffer)

    def reset(self) -> None:
        self._buffer.clear()

    def _retain_possible_marker_prefix(self) -> None:
        keep = 0
        upper = min(len(self._buffer), len(DFDMD_MARKER) - 1)
        for length in range(upper, 0, -1):
            if DFDMD_MARKER.startswith(self._buffer[-length:]):
                keep = length
                break
        discard = len(self._buffer) - keep
        if discard > 0:
            self.discarded_bytes += discard
            del self._buffer[:discard]

    def feed(
        self, data: Union[bytes, bytearray, memoryview]
    ) -> List[C4001Measurement]:
        if not isinstance(data, (bytes, bytearray, memoryview)):
            raise TypeError('data must be bytes-like')
        if not data:
            return []

        self._buffer.extend(data)
        measurements: List[C4001Measurement] = []

        while self._buffer:
            start = self._buffer.find(DFDMD_MARKER)
            if start < 0:
                self._retain_possible_marker_prefix()
                break
            if start > 0:
                self.discarded_bytes += start
                del self._buffer[:start]

            end = self._buffer.find(FRAME_END, len(DFDMD_MARKER))
            next_start = self._buffer.find(DFDMD_MARKER, len(DFDMD_MARKER))
            if next_start >= 0 and (end < 0 or next_start < end):
                # The first frame was truncated.  Resynchronise at the newer
                # complete-looking marker instead of sacrificing both frames.
                self.malformed_frames += 1
                self.discarded_bytes += next_start
                del self._buffer[:next_start]
                continue

            if end < 0:
                if len(self._buffer) > self.max_frame_bytes:
                    self.malformed_frames += 1
                    self.discarded_bytes += 1
                    del self._buffer[0]
                    continue
                break

            frame_length = end + 1
            frame = bytes(self._buffer[:frame_length])
            del self._buffer[:frame_length]
            if frame_length > self.max_frame_bytes:
                self.malformed_frames += 1
                self.discarded_bytes += frame_length
                continue

            try:
                measurements.append(parse_dfdmd_frame(frame))
            except C4001FrameError:
                self.malformed_frames += 1

        return measurements


def encode_command(command: str) -> bytes:
    """Encode one command exactly as the official driver writes it."""

    if not isinstance(command, str):
        raise TypeError('command must be str')
    command = command.strip()
    if not command:
        raise ValueError('command must not be empty')
    if '\r' in command or '\n' in command:
        raise ValueError('command must not contain a line break')
    try:
        return command.encode('ascii')
    except UnicodeEncodeError as error:
        raise ValueError('command must be ASCII') from error


def _format_number(value: float) -> str:
    return format(float(value), '.12g')


def build_speed_mode_configuration(
    min_range_m: float,
    max_range_m: float,
    threshold_factor: int,
    micro_motion_enabled: bool,
) -> List[str]:
    """Return the official stop/configure/save/start command sequence."""

    min_range_m = float(min_range_m)
    max_range_m = float(max_range_m)
    threshold_factor = int(threshold_factor)
    if not math.isfinite(min_range_m) or not math.isfinite(max_range_m):
        raise ValueError('detection ranges must be finite')
    if not 0.0 <= min_range_m < max_range_m <= MAX_PROTOCOL_DISTANCE_M:
        raise ValueError('detection range must satisfy 0 <= min < max <= 25m')
    if not 0 <= threshold_factor <= 65535:
        raise ValueError('threshold_factor must be between 0 and 65535')

    return [
        'sensorStop',
        'setRunApp 1',
        f'setRange {_format_number(min_range_m)} {_format_number(max_range_m)}',
        f'setThrFactor {threshold_factor}',
        f'setMicroMotion {1 if micro_motion_enabled else 0}',
        'saveConfig',
        'sensorStart',
    ]
