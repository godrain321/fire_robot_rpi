import math

import pytest

from inno_mmwave.c4001_protocol import (
    C4001FrameError,
    C4001StreamParser,
    build_speed_mode_configuration,
    encode_command,
    parse_dfdmd_frame,
)


def test_parse_official_detected_example():
    sample = parse_dfdmd_frame(
        b'$DFDMD,1,1,1.817,0.129,15304, , *\r\n'
    )
    assert sample.detected
    assert sample.target_count == 1
    assert sample.target_id == 1
    assert sample.distance_m == pytest.approx(1.817)
    assert sample.speed_mps == pytest.approx(0.129)
    assert sample.energy == 15304


def test_parse_no_target_placeholders():
    sample = parse_dfdmd_frame('$DFDMD,0, , , , , , *')
    assert not sample.detected
    assert sample.target_id is None
    assert sample.distance_m is None
    assert sample.speed_mps is None
    assert sample.energy is None


def test_parse_tolerates_prompt_and_signed_speed():
    sample = parse_dfdmd_frame(
        'DFRobot:/>$DFDMD, 1, 1, 0.613, -0.179, 75490, , *\r\n'
    )
    assert sample.distance_m == pytest.approx(0.613)
    assert sample.speed_mps == pytest.approx(-0.179)


@pytest.mark.parametrize(
    'frame',
    [
        '$DFDMD,2,1,1.0,0.1,5, , *',
        '$DFDMD,1,2,1.0,0.1,5, , *',
        '$DFDMD,1,1,nan,0.1,5, , *',
        '$DFDMD,1,1,1.0,11.0,5, , *',
        '$DFDMD,1,1,1.0,0.1,-1, , *',
        '$DFDMD,1,1,1.0,0.1,5,*',
        '$DFDMD,1,1,1.0,0.1,5, , ',
    ],
)
def test_rejects_malformed_or_out_of_range_frames(frame):
    with pytest.raises(C4001FrameError):
        parse_dfdmd_frame(frame)


def test_incremental_parser_handles_noise_fragments_and_multiple_frames():
    parser = C4001StreamParser()
    assert parser.feed(b'boot\r\nDFRobot:/>$DF') == []
    first_half = parser.feed(b'DMD,1,1,2.40,-0.20,321, , ')
    assert first_half == []
    samples = parser.feed(
        b'*\r\nDFRobot:/>$DFDMD,0, , , , , , *\r\n'
    )
    assert len(samples) == 2
    assert samples[0].distance_m == pytest.approx(2.4)
    assert samples[1].target_count == 0
    assert parser.buffered_bytes == 0


def test_incremental_parser_resynchronises_after_truncated_frame():
    parser = C4001StreamParser()
    samples = parser.feed(
        b'$DFDMD,1,1,broken$DFDMD,1,1,3.2,0.0,99, , *'
    )
    assert len(samples) == 1
    assert samples[0].distance_m == pytest.approx(3.2)
    assert parser.malformed_frames == 1


def test_incremental_parser_bounds_unterminated_input():
    parser = C4001StreamParser(max_frame_bytes=48)
    assert parser.feed(b'$DFDMD,' + b'9' * 100) == []
    assert parser.buffered_bytes < 48
    assert parser.malformed_frames >= 1


def test_encode_command_matches_official_raw_write_and_rejects_injection():
    assert encode_command('sensorStart') == b'sensorStart'
    with pytest.raises(ValueError):
        encode_command('sensorStop\r\nresetCfg')


def test_speed_mode_configuration_sequence():
    commands = build_speed_mode_configuration(1.2, 12.0, 10, True)
    assert commands == [
        'sensorStop',
        'setRunApp 1',
        'setRange 1.2 12',
        'setThrFactor 10',
        'setMicroMotion 1',
        'saveConfig',
        'sensorStart',
    ]
    assert all(math.isfinite(value) for value in (1.2, 12.0))


def test_configuration_validation():
    with pytest.raises(ValueError):
        build_speed_mode_configuration(12.0, 1.2, 10, True)
    with pytest.raises(ValueError):
        build_speed_mode_configuration(1.2, 12.0, 70000, True)
