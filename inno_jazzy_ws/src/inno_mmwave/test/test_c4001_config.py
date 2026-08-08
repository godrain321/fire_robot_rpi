import math

import pytest

from inno_mmwave.c4001_config import (
    BoundedResponseParser,
    C4001ConfigError,
    C4001DesiredConfig,
    C4001Readback,
    C4001ResponseError,
    C4001ResponseOverflow,
    CommandStep,
    CommandTiming,
    ResponseKind,
    build_official_configuration_plan,
    build_official_readback_plan,
    parse_response_numbers,
    readback_from_values,
    verify_readback,
)


def desired_config():
    return C4001DesiredConfig(
        min_range_m=1.2,
        max_range_m=12.0,
        threshold_factor=10,
        micro_motion_enabled=True,
    )


def test_configuration_plan_preserves_official_three_cycles():
    plan = build_official_configuration_plan(desired_config())
    by_phase = {}
    for step in plan.steps:
        by_phase.setdefault(step.phase, []).append(step.command)

    assert by_phase == {
        'mode': ['sensorStop', 'setRunApp 1', 'saveConfig', 'sensorStart'],
        'range_threshold': [
            'sensorStop',
            'setRange 1.2 12',
            'setThrFactor 10',
            'saveConfig',
            'sensorStart',
        ],
        'micro_motion': [
            'sensorStop',
            'setMicroMotion 1',
            'saveConfig',
            'sensorStart',
        ],
    }
    assert all('\r' not in item.command and '\n' not in item.command
               for item in plan.steps)


def test_stop_steps_are_bounded_retries_with_echo_expectation():
    plan = build_official_configuration_plan(
        desired_config(),
        CommandTiming(stop_max_attempts=4, stop_retry_interval_sec=0.25),
    )
    stops = [step for step in plan.steps if step.command == 'sensorStop']
    assert len(stops) == 3
    for step in stops:
        assert step.max_attempts == 4
        assert step.retry_interval_sec == pytest.approx(0.25)
        assert step.clear_input_before
        assert step.expected_response is not None
        assert step.expected_response.kind is ResponseKind.CONTAINS
        assert step.expected_response.marker == b'sensorStop'


def test_micro_motion_off_is_encoded_as_zero():
    desired = C4001DesiredConfig(1.2, 6.0, 12, False)
    commands = [
        step.command for step in build_official_configuration_plan(desired).steps
    ]
    assert 'setMicroMotion 0' in commands


def test_readback_plan_has_three_official_query_cycles_and_mode_evidence():
    plan = build_official_readback_plan()
    assert [step.command for step in plan.steps] == [
        'sensorStop', 'getRange', 'sensorStart',
        'sensorStop', 'getThrFactor', 'sensorStart',
        'sensorStop', 'getMicroMotion', 'sensorStart',
    ]
    queries = [step for step in plan.steps if step.command.startswith('get')]
    assert [item.expected_response.value_count for item in queries] == [2, 1, 1]
    assert [item.expected_response.readback_key for item in queries] == [
        'range_m', 'threshold_factor', 'micro_motion_enabled'
    ]
    final = plan.steps[-1].expected_response
    assert final is not None
    assert final.kind is ResponseKind.ACTIVE_FRAME
    assert final.marker == b'$DFDMD,'


def test_plan_and_timing_validation_reject_unsafe_values():
    with pytest.raises(ValueError, match='line endings'):
        CommandStep('bad', 'sensorStart\r\n', 0.1)
    with pytest.raises(ValueError, match='max_attempts'):
        CommandStep('bad', 'sensorStart', 0.1, max_attempts=0)
    with pytest.raises(ValueError, match='retries require'):
        CommandStep('bad', 'sensorStart', 0.1, max_attempts=2)
    with pytest.raises(ValueError, match='timing'):
        CommandTiming(command_settle_sec=-0.1)


@pytest.mark.parametrize(
    'kwargs',
    [
        dict(min_range_m=2.0, max_range_m=2.0,
             threshold_factor=10, micro_motion_enabled=True),
        dict(min_range_m=math.nan, max_range_m=12.0,
             threshold_factor=10, micro_motion_enabled=True),
        dict(min_range_m=1.2, max_range_m=12.0,
             threshold_factor=65536, micro_motion_enabled=True),
    ],
)
def test_desired_config_validation(kwargs):
    with pytest.raises((ValueError, TypeError)):
        C4001DesiredConfig(**kwargs)


def test_parse_complete_response_without_any_line_ending():
    assert parse_response_numbers('DFRobot:/>Response 1.2 12', 2) == (1.2, 12.0)
    assert parse_response_numbers(b'noise Response -3.5e-1', 1) == (-0.35,)


def test_parse_response_requires_marker_and_expected_value_count():
    with pytest.raises(C4001ResponseError, match='missing'):
        parse_response_numbers(b'1.2 12.0', 2)
    with pytest.raises(C4001ResponseError, match='2 complete'):
        parse_response_numbers(b'Response 1.2', 2)
    with pytest.raises(ValueError, match='positive'):
        parse_response_numbers(b'Response 1', 0)


def test_incremental_parser_handles_fragmented_marker_and_values():
    parser = BoundedResponseParser(expected_count=2, max_buffer_bytes=64)
    assert parser.feed(b'echo getRange DFRobot:/>Res') is None
    assert parser.feed(b'ponse 1.') is None
    assert parser.feed(b'2 12') is None
    # A delimiter, not specifically CR/LF, establishes the final token edge.
    assert parser.feed(b' DFRobot:/>') == (1.2, 12.0)
    assert parser.buffered_bytes == 0
    assert parser.discarded_bytes > 0


def test_incremental_parser_finalize_accepts_stream_end_as_boundary():
    parser = BoundedResponseParser(expected_count=1)
    assert parser.feed(b'Response 10') is None
    assert parser.finalize() == (10.0,)
    assert parser.buffered_bytes == 0


def test_incremental_parser_does_not_accept_partial_final_number():
    parser = BoundedResponseParser(expected_count=2)
    assert parser.feed(b'Response 1.2 1') is None
    assert parser.feed(b'2.0 ') == (1.2, 12.0)


def test_incremental_parser_bounds_unmarked_noise():
    parser = BoundedResponseParser(expected_count=1, max_buffer_bytes=32)
    for _ in range(20):
        assert parser.feed(b'x' * 20) is None
        assert parser.buffered_bytes < len(b'Response')
    assert parser.discarded_bytes >= 400


def test_incremental_parser_raises_when_marked_response_exceeds_bound():
    parser = BoundedResponseParser(expected_count=2, max_buffer_bytes=24)
    with pytest.raises(C4001ResponseOverflow):
        parser.feed(b'Response ' + b'9' * 40)
    assert parser.buffered_bytes == 0
    assert parser.overflow_count == 1


def test_incremental_parser_rejects_invalid_types_and_incomplete_finalize():
    parser = BoundedResponseParser(expected_count=1)
    with pytest.raises(TypeError):
        parser.feed('Response 1')
    parser.feed(b'Response nope')
    with pytest.raises(C4001ResponseError, match='incomplete'):
        parser.finalize()


def test_readback_conversion_and_verification_success_with_range_tolerance():
    actual = readback_from_values(
        (1.23, 11.97), (10.0,), (1.0,), speed_mode_confirmed=True
    )
    result = verify_readback(
        desired_config(), actual, range_tolerance_m=0.05
    )
    assert result.verified
    assert result.mismatches == ()
    assert result.summary == 'VERIFIED'


def test_readback_conversion_rejects_invalid_discrete_values():
    with pytest.raises(C4001ResponseError, match='threshold'):
        readback_from_values((1.2, 12.0), (10.5,), (1,),
                             speed_mode_confirmed=True)
    with pytest.raises(C4001ResponseError, match='micro-motion'):
        readback_from_values((1.2, 12.0), (10,), (2,),
                             speed_mode_confirmed=True)
    with pytest.raises(C4001ResponseError, match='exactly two'):
        readback_from_values((1.2,), (10,), (1,),
                             speed_mode_confirmed=True)


def test_verification_reports_all_missing_and_mismatched_fields():
    actual = C4001Readback(
        min_range_m=0.4,
        max_range_m=None,
        threshold_factor=20,
        micro_motion_enabled=False,
        speed_mode_confirmed=False,
    )
    result = verify_readback(desired_config(), actual)
    assert not result.verified
    assert len(result.mismatches) == 5
    assert 'min_range_m expected 1.2, got 0.4' in result.mismatches
    assert 'max_range_m=MISSING' in result.mismatches
    assert 'threshold_factor expected 10, got 20' in result.mismatches
    assert 'micro_motion_enabled expected True, got False' in result.mismatches
    assert 'speed_mode_confirmed expected True, got False' in result.mismatches


def test_verification_rejects_invalid_tolerance_and_argument_types():
    with pytest.raises(ValueError, match='tolerance'):
        verify_readback(desired_config(), C4001Readback(),
                        range_tolerance_m=-0.1)
    with pytest.raises(TypeError):
        verify_readback(desired_config(), object())


def test_base_error_is_a_value_error_for_simple_node_handling():
    assert issubclass(C4001ConfigError, ValueError)
