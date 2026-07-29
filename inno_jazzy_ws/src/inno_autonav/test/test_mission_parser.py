import pytest

from inno_autonav.mission_commander import normalize_label, parse_mission


@pytest.mark.parametrize(
    'text, expected',
    [
        ('go exit2', (None, 'exit2')),
        ('go exit1 exit2', ('exit1', 'exit2')),
        ('exit1에서 exit2로가', ('exit1', 'exit2')),
        ('EXIT1 to E2', ('EXIT1', 'E2')),
    ],
)
def test_parse_mission(text, expected):
    assert parse_mission(text) == expected


def test_alias_normalization():
    assert normalize_label('E1') == 'exit1'
    assert normalize_label('EXIT2') == 'exit2'
    assert normalize_label('door', {'door': 'exit3'}) == 'exit3'


def test_bad_mission_rejected():
    with pytest.raises(ValueError):
        parse_mission('go exit1 exit2 exit3')
