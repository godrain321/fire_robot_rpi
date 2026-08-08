"""Pure helpers for configuring and verifying a C4001 over its ASCII CLI.

The official DFRobot UART driver configures speed mode, range/threshold and
micro-motion in three separate stop/save/start cycles.  It also reads values
back by stopping the sensor, issuing one ``get...`` command and starting the
sensor again.  This module describes those operations without opening a
serial port, so a ROS node can execute them with a non-blocking state machine.

The module deliberately makes no line-ending assumption.  C4001 commands are
raw ASCII, while numeric readback is recognised from the ``Response`` marker
and the expected number of whitespace-separated numeric values.
"""

from dataclasses import dataclass
from enum import Enum
import math
import re
from typing import Optional, Sequence, Tuple, Union


RESPONSE_MARKER = b'Response'
SPEED_FRAME_MARKER = b'$DFDMD,'
_NUMBER = rb'[+-]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][+-]?\d+)?'
_NUMBER_TEXT = re.compile(
    r'^[+-]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][+-]?\d+)?$'
)


class C4001ConfigError(ValueError):
    """Base exception for invalid plans and configuration responses."""


class C4001ResponseError(C4001ConfigError):
    """Raised when a C4001 configuration response cannot be parsed."""


class C4001ResponseOverflow(C4001ResponseError):
    """Raised when a response beginning with ``Response`` exceeds its bound."""


class ResponseKind(str, Enum):
    """Type of response an asynchronous command executor should await."""

    CONTAINS = 'contains'
    NUMBERS = 'numbers'
    ACTIVE_FRAME = 'active_frame'


@dataclass(frozen=True)
class ResponseExpectation:
    """Expected UART evidence after a command.

    ``value_count`` is used only for :attr:`ResponseKind.NUMBERS`.
    ``readback_key`` lets a node associate parsed values with a desired
    setting without relying on the command string.
    """

    kind: ResponseKind
    marker: bytes
    timeout_sec: float
    value_count: int = 0
    readback_key: Optional[str] = None

    def __post_init__(self) -> None:
        if not isinstance(self.kind, ResponseKind):
            raise TypeError('kind must be a ResponseKind')
        if not isinstance(self.marker, bytes) or not self.marker:
            raise ValueError('response marker must be non-empty bytes')
        try:
            self.marker.decode('ascii')
        except UnicodeDecodeError as error:
            raise ValueError('response marker must be ASCII') from error
        if not math.isfinite(self.timeout_sec) or self.timeout_sec <= 0.0:
            raise ValueError('response timeout must be finite and positive')
        if self.kind is ResponseKind.NUMBERS:
            if self.marker != RESPONSE_MARKER:
                raise ValueError('numeric responses must use the Response marker')
            if self.value_count <= 0:
                raise ValueError('numeric responses need at least one value')
            if not self.readback_key:
                raise ValueError('numeric responses need a readback key')
        elif self.value_count != 0 or self.readback_key is not None:
            raise ValueError(
                'only numeric responses may define values or a readback key'
            )


@dataclass(frozen=True)
class CommandStep:
    """One raw-ASCII command in a non-blocking C4001 execution plan."""

    phase: str
    command: str
    delay_after_sec: float
    expected_response: Optional[ResponseExpectation] = None
    max_attempts: int = 1
    retry_interval_sec: float = 0.0
    clear_input_before: bool = False

    def __post_init__(self) -> None:
        if not isinstance(self.phase, str) or not self.phase.strip():
            raise ValueError('command phase must not be empty')
        if not isinstance(self.command, str) or not self.command.strip():
            raise ValueError('command must not be empty')
        if '\r' in self.command or '\n' in self.command:
            raise ValueError('C4001 commands must not contain line endings')
        try:
            self.command.encode('ascii')
        except UnicodeEncodeError as error:
            raise ValueError('C4001 commands must be ASCII') from error
        if (
            not math.isfinite(self.delay_after_sec)
            or self.delay_after_sec < 0.0
        ):
            raise ValueError('command delay must be finite and non-negative')
        if isinstance(self.max_attempts, bool) or self.max_attempts < 1:
            raise ValueError('max_attempts must be at least one')
        if (
            not math.isfinite(self.retry_interval_sec)
            or self.retry_interval_sec < 0.0
        ):
            raise ValueError('retry interval must be finite and non-negative')
        if self.max_attempts > 1 and self.expected_response is None:
            raise ValueError('retries require an expected response')


@dataclass(frozen=True)
class CommandPlan:
    """Named immutable sequence of C4001 commands."""

    name: str
    steps: Tuple[CommandStep, ...]

    def __post_init__(self) -> None:
        if not isinstance(self.name, str) or not self.name.strip():
            raise ValueError('plan name must not be empty')
        if not isinstance(self.steps, tuple) or not self.steps:
            raise ValueError('command plan must contain a tuple of steps')
        if not all(isinstance(step, CommandStep) for step in self.steps):
            raise TypeError('command plan contains a non-CommandStep value')


@dataclass(frozen=True)
class CommandTiming:
    """Conservative UART timing used to build command plans."""

    command_settle_sec: float = 0.10
    stop_settle_sec: float = 1.0
    save_settle_sec: float = 0.80
    start_settle_sec: float = 0.10
    stop_response_timeout_sec: float = 1.5
    query_response_timeout_sec: float = 1.0
    mode_frame_timeout_sec: float = 1.5
    stop_max_attempts: int = 3
    stop_retry_interval_sec: float = 0.40

    def __post_init__(self) -> None:
        non_negative = (
            self.command_settle_sec,
            self.stop_settle_sec,
            self.save_settle_sec,
            self.start_settle_sec,
            self.stop_retry_interval_sec,
        )
        if any(not math.isfinite(value) or value < 0.0 for value in non_negative):
            raise ValueError('command timing values must be finite and non-negative')
        positive = (
            self.stop_response_timeout_sec,
            self.query_response_timeout_sec,
            self.mode_frame_timeout_sec,
        )
        if any(not math.isfinite(value) or value <= 0.0 for value in positive):
            raise ValueError('response timeouts must be finite and positive')
        if (
            isinstance(self.stop_max_attempts, bool)
            or self.stop_max_attempts < 1
        ):
            raise ValueError('stop_max_attempts must be at least one')


@dataclass(frozen=True)
class C4001DesiredConfig:
    """Desired configuration for the C4001 speed/range application."""

    min_range_m: float
    max_range_m: float
    threshold_factor: int
    micro_motion_enabled: bool

    def __post_init__(self) -> None:
        if (
            not math.isfinite(self.min_range_m)
            or not math.isfinite(self.max_range_m)
            or self.min_range_m < 0.0
            or self.min_range_m >= self.max_range_m
        ):
            raise ValueError('desired range must satisfy 0 <= min < max')
        if (
            isinstance(self.threshold_factor, bool)
            or not isinstance(self.threshold_factor, int)
            or not 0 <= self.threshold_factor <= 65535
        ):
            raise ValueError('threshold_factor must be an integer from 0 to 65535')
        if not isinstance(self.micro_motion_enabled, bool):
            raise TypeError('micro_motion_enabled must be bool')


@dataclass(frozen=True)
class C4001Readback:
    """Actual values obtained from the three official readback commands."""

    min_range_m: Optional[float] = None
    max_range_m: Optional[float] = None
    threshold_factor: Optional[int] = None
    micro_motion_enabled: Optional[bool] = None
    speed_mode_confirmed: Optional[bool] = None

    def __post_init__(self) -> None:
        for name in ('min_range_m', 'max_range_m'):
            value = getattr(self, name)
            if value is not None and not math.isfinite(value):
                raise ValueError(f'{name} must be finite when present')
        if self.threshold_factor is not None and (
            isinstance(self.threshold_factor, bool)
            or not isinstance(self.threshold_factor, int)
            or not 0 <= self.threshold_factor <= 65535
        ):
            raise ValueError('readback threshold must be an integer from 0 to 65535')
        for name in ('micro_motion_enabled', 'speed_mode_confirmed'):
            value = getattr(self, name)
            if value is not None and not isinstance(value, bool):
                raise TypeError(f'{name} must be bool or None')


@dataclass(frozen=True)
class ConfigVerification:
    """Result of comparing a desired configuration with hardware readback."""

    verified: bool
    mismatches: Tuple[str, ...]

    @property
    def summary(self) -> str:
        if self.verified:
            return 'VERIFIED'
        return '; '.join(self.mismatches)


def _format_number(value: float) -> str:
    return format(float(value), '.12g')


def _stop_step(phase: str, timing: CommandTiming) -> CommandStep:
    return CommandStep(
        phase=phase,
        command='sensorStop',
        delay_after_sec=timing.stop_settle_sec,
        expected_response=ResponseExpectation(
            kind=ResponseKind.CONTAINS,
            marker=b'sensorStop',
            timeout_sec=timing.stop_response_timeout_sec,
        ),
        max_attempts=timing.stop_max_attempts,
        retry_interval_sec=timing.stop_retry_interval_sec,
        clear_input_before=True,
    )


def _plain_step(
    phase: str,
    command: str,
    delay_after_sec: float,
    *,
    clear_input_before: bool = False,
) -> CommandStep:
    return CommandStep(
        phase=phase,
        command=command,
        delay_after_sec=delay_after_sec,
        clear_input_before=clear_input_before,
    )


def build_official_configuration_plan(
    desired: C4001DesiredConfig,
    timing: Optional[CommandTiming] = None,
) -> CommandPlan:
    """Build the official three-cycle speed-mode configuration plan.

    DFRobot's UART implementation saves/starts after selecting speed mode,
    performs range and threshold together in a second cycle, and configures
    micro-motion in a third cycle.  Keeping these boundaries is important
    because range settings are application-mode specific.
    """

    if not isinstance(desired, C4001DesiredConfig):
        raise TypeError('desired must be C4001DesiredConfig')
    timing = timing or CommandTiming()
    if not isinstance(timing, CommandTiming):
        raise TypeError('timing must be CommandTiming')

    steps = (
        _stop_step('mode', timing),
        _plain_step('mode', 'setRunApp 1', timing.command_settle_sec),
        _plain_step('mode', 'saveConfig', timing.save_settle_sec),
        _plain_step('mode', 'sensorStart', timing.start_settle_sec),
        _stop_step('range_threshold', timing),
        _plain_step(
            'range_threshold',
            'setRange '
            f'{_format_number(desired.min_range_m)} '
            f'{_format_number(desired.max_range_m)}',
            timing.command_settle_sec,
        ),
        _plain_step(
            'range_threshold',
            f'setThrFactor {desired.threshold_factor}',
            timing.command_settle_sec,
        ),
        _plain_step('range_threshold', 'saveConfig', timing.save_settle_sec),
        _plain_step('range_threshold', 'sensorStart', timing.start_settle_sec),
        _stop_step('micro_motion', timing),
        _plain_step(
            'micro_motion',
            f'setMicroMotion {1 if desired.micro_motion_enabled else 0}',
            timing.command_settle_sec,
        ),
        _plain_step('micro_motion', 'saveConfig', timing.save_settle_sec),
        _plain_step('micro_motion', 'sensorStart', timing.start_settle_sec),
    )
    return CommandPlan('official_speed_mode_configuration', steps)


def build_official_readback_plan(
    timing: Optional[CommandTiming] = None,
) -> CommandPlan:
    """Build official stop/query/start cycles for all verifiable settings.

    The final ``sensorStart`` expects a ``$DFDMD`` active frame.  The official
    UART API has no separate getter for ``setRunApp``; frame type is therefore
    the hardware evidence that speed/range mode is active.
    """

    timing = timing or CommandTiming()
    if not isinstance(timing, CommandTiming):
        raise TypeError('timing must be CommandTiming')

    definitions = (
        ('read_range', 'getRange', 2, 'range_m'),
        ('read_threshold', 'getThrFactor', 1, 'threshold_factor'),
        ('read_micro_motion', 'getMicroMotion', 1, 'micro_motion_enabled'),
    )
    steps = []
    for index, (phase, command, count, key) in enumerate(definitions):
        steps.append(_stop_step(phase, timing))
        steps.append(CommandStep(
            phase=phase,
            command=command,
            delay_after_sec=timing.command_settle_sec,
            expected_response=ResponseExpectation(
                kind=ResponseKind.NUMBERS,
                marker=RESPONSE_MARKER,
                timeout_sec=timing.query_response_timeout_sec,
                value_count=count,
                readback_key=key,
            ),
            clear_input_before=True,
        ))
        is_last = index == len(definitions) - 1
        expectation = None
        if is_last:
            expectation = ResponseExpectation(
                kind=ResponseKind.ACTIVE_FRAME,
                marker=SPEED_FRAME_MARKER,
                timeout_sec=timing.mode_frame_timeout_sec,
            )
        steps.append(CommandStep(
            phase=phase,
            command='sensorStart',
            delay_after_sec=timing.start_settle_sec,
            expected_response=expectation,
            clear_input_before=True,
        ))
    return CommandPlan('official_speed_mode_readback', tuple(steps))


def _coerce_bytes(data: Union[str, bytes, bytearray, memoryview]) -> bytes:
    if isinstance(data, str):
        try:
            return data.encode('ascii')
        except UnicodeEncodeError as error:
            raise C4001ResponseError('response is not ASCII') from error
    if not isinstance(data, (bytes, bytearray, memoryview)):
        raise TypeError('response data must be str or bytes-like')
    return bytes(data)


def _response_pattern(expected_count: int, allow_end: bool) -> re.Pattern:
    if isinstance(expected_count, bool) or expected_count <= 0:
        raise ValueError('expected_count must be a positive integer')
    pieces = [re.escape(RESPONSE_MARKER), rb'\s+']
    for index in range(expected_count):
        if index:
            pieces.append(rb'\s+')
        pieces.extend((b'(', _NUMBER, b')'))
    # A streaming parser must see a delimiter after the last number so that
    # ``12`` is not accepted just before a later chunk extends it to ``12.0``.
    pieces.append(rb'(?=$|[\s$/:>,*])' if allow_end else rb'(?=[\s$/:>,*])')
    return re.compile(b''.join(pieces))


def _match_response_numbers(
    data: bytes,
    expected_count: int,
    *,
    allow_end: bool,
) -> Optional[Tuple[Tuple[float, ...], int]]:
    match = _response_pattern(expected_count, allow_end).search(data)
    if match is None:
        return None
    values = []
    for group in match.groups():
        text = group.decode('ascii')
        if _NUMBER_TEXT.fullmatch(text) is None:
            return None
        value = float(text)
        if not math.isfinite(value):
            raise C4001ResponseError('Response contains a non-finite number')
        values.append(value)
    return tuple(values), match.end()


def parse_response_numbers(
    data: Union[str, bytes, bytearray, memoryview],
    expected_count: int,
) -> Tuple[float, ...]:
    """Parse a complete ``Response`` record without assuming a line ending."""

    raw = _coerce_bytes(data)
    matched = _match_response_numbers(raw, expected_count, allow_end=True)
    if matched is None:
        if RESPONSE_MARKER not in raw:
            raise C4001ResponseError('missing Response marker')
        raise C4001ResponseError(
            f'Response does not contain {expected_count} complete numeric values'
        )
    return matched[0]


class BoundedResponseParser:
    """Incrementally parse one bounded C4001 numeric readback response.

    :meth:`feed` completes only after a delimiter is observed after the final
    numeric value.  If a device stops transmitting exactly at that value, the
    executor may call :meth:`finalize` after its response timeout/idle period.
    Neither method requires CR, LF, or any other particular record terminator.
    """

    def __init__(self, expected_count: int, max_buffer_bytes: int = 512) -> None:
        if isinstance(expected_count, bool) or expected_count <= 0:
            raise ValueError('expected_count must be a positive integer')
        if max_buffer_bytes < len(RESPONSE_MARKER) + 4:
            raise ValueError('max_buffer_bytes is too small')
        self.expected_count = int(expected_count)
        self.max_buffer_bytes = int(max_buffer_bytes)
        self._buffer = bytearray()
        self.discarded_bytes = 0
        self.overflow_count = 0

    @property
    def buffered_bytes(self) -> int:
        return len(self._buffer)

    def reset(self) -> None:
        self._buffer.clear()

    def _retain_possible_marker(self) -> None:
        keep = 0
        upper = min(len(self._buffer), len(RESPONSE_MARKER) - 1)
        for length in range(upper, 0, -1):
            if RESPONSE_MARKER.startswith(self._buffer[-length:]):
                keep = length
                break
        discard = len(self._buffer) - keep
        if discard > 0:
            self.discarded_bytes += discard
            del self._buffer[:discard]

    def _bound_and_align(self) -> None:
        marker = self._buffer.find(RESPONSE_MARKER)
        if marker < 0:
            self._retain_possible_marker()
            return
        if marker > 0:
            self.discarded_bytes += marker
            del self._buffer[:marker]
        if len(self._buffer) > self.max_buffer_bytes:
            self.overflow_count += 1
            self.discarded_bytes += len(self._buffer)
            self._buffer.clear()
            raise C4001ResponseOverflow(
                f'Response exceeded {self.max_buffer_bytes} bytes'
            )

    def feed(
        self,
        data: Union[bytes, bytearray, memoryview],
    ) -> Optional[Tuple[float, ...]]:
        if not isinstance(data, (bytes, bytearray, memoryview)):
            raise TypeError('stream data must be bytes-like')
        if not data:
            return None
        self._buffer.extend(data)
        self._bound_and_align()
        matched = _match_response_numbers(
            bytes(self._buffer), self.expected_count, allow_end=False
        )
        if matched is None:
            return None
        values, _ = matched
        self._buffer.clear()
        return values

    def finalize(self) -> Tuple[float, ...]:
        """Finish a response whose final numeric token ended with the stream."""

        matched = _match_response_numbers(
            bytes(self._buffer), self.expected_count, allow_end=True
        )
        self._buffer.clear()
        if matched is None:
            raise C4001ResponseError('incomplete numeric Response')
        return matched[0]


def readback_from_values(
    range_values: Sequence[float],
    threshold_values: Sequence[float],
    micro_motion_values: Sequence[float],
    *,
    speed_mode_confirmed: bool,
) -> C4001Readback:
    """Convert parsed query values into a strongly typed readback object."""

    if len(range_values) != 2:
        raise C4001ResponseError('getRange must return exactly two values')
    if len(threshold_values) != 1:
        raise C4001ResponseError('getThrFactor must return exactly one value')
    if len(micro_motion_values) != 1:
        raise C4001ResponseError('getMicroMotion must return exactly one value')

    minimum, maximum = (float(value) for value in range_values)
    threshold = float(threshold_values[0])
    micro_motion = float(micro_motion_values[0])
    values = (minimum, maximum, threshold, micro_motion)
    if any(not math.isfinite(value) for value in values):
        raise C4001ResponseError('readback contains a non-finite value')
    if not threshold.is_integer() or not 0 <= threshold <= 65535:
        raise C4001ResponseError('threshold readback is not a valid integer')
    if micro_motion not in (0.0, 1.0):
        raise C4001ResponseError('micro-motion readback must be 0 or 1')
    if not isinstance(speed_mode_confirmed, bool):
        raise TypeError('speed_mode_confirmed must be bool')

    return C4001Readback(
        min_range_m=minimum,
        max_range_m=maximum,
        threshold_factor=int(threshold),
        micro_motion_enabled=bool(micro_motion),
        speed_mode_confirmed=speed_mode_confirmed,
    )


def verify_readback(
    desired: C4001DesiredConfig,
    actual: C4001Readback,
    *,
    range_tolerance_m: float = 0.05,
) -> ConfigVerification:
    """Compare desired values to readback, reporting every missing/mismatched field."""

    if not isinstance(desired, C4001DesiredConfig):
        raise TypeError('desired must be C4001DesiredConfig')
    if not isinstance(actual, C4001Readback):
        raise TypeError('actual must be C4001Readback')
    if not math.isfinite(range_tolerance_m) or range_tolerance_m < 0.0:
        raise ValueError('range_tolerance_m must be finite and non-negative')

    mismatches = []
    for name in ('min_range_m', 'max_range_m'):
        expected_value = getattr(desired, name)
        actual_value = getattr(actual, name)
        if actual_value is None:
            mismatches.append(f'{name}=MISSING')
        elif abs(actual_value - expected_value) > range_tolerance_m:
            mismatches.append(
                f'{name} expected {expected_value:g}, got {actual_value:g}'
            )

    if actual.threshold_factor is None:
        mismatches.append('threshold_factor=MISSING')
    elif actual.threshold_factor != desired.threshold_factor:
        mismatches.append(
            'threshold_factor expected '
            f'{desired.threshold_factor}, got {actual.threshold_factor}'
        )

    if actual.micro_motion_enabled is None:
        mismatches.append('micro_motion_enabled=MISSING')
    elif actual.micro_motion_enabled != desired.micro_motion_enabled:
        mismatches.append(
            'micro_motion_enabled expected '
            f'{desired.micro_motion_enabled}, got {actual.micro_motion_enabled}'
        )

    if actual.speed_mode_confirmed is None:
        mismatches.append('speed_mode_confirmed=MISSING')
    elif not actual.speed_mode_confirmed:
        mismatches.append('speed_mode_confirmed expected True, got False')

    result = tuple(mismatches)
    return ConfigVerification(verified=not result, mismatches=result)
