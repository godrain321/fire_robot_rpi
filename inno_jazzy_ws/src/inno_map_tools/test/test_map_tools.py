from pathlib import Path

from PIL import Image
import pytest
import yaml

from inno_map_tools.build_no_go_mask import build
from inno_map_tools.map_utils import map_to_pixel, pixel_to_map


def test_coordinate_conversion_round_trip():
    pixel = map_to_pixel(1.0, 2.0, -1.0, -2.0, 0.05, 200)
    assert pixel == (40, 119)
    map_point = pixel_to_map(*pixel, -1.0, -2.0, 0.05, 200)
    assert map_point == pytest.approx((1.0, 2.0))


def test_build_preserves_raw_map_and_fills_polygon(tmp_path: Path):
    raw_path = tmp_path / 'raw.pgm'
    Image.new('L', (20, 20), 254).save(raw_path, format='PPM')
    raw_bytes = raw_path.read_bytes()
    map_yaml = tmp_path / 'raw.yaml'
    map_yaml.write_text(
        yaml.safe_dump(
            {
                'image': 'raw.pgm',
                'mode': 'trinary',
                'resolution': 0.1,
                'origin': [0.0, 0.0, 0.0],
                'negate': 0,
                'occupied_thresh': 0.65,
                'free_thresh': 0.196,
            },
            sort_keys=False,
        ),
        encoding='utf-8',
    )
    zones_yaml = tmp_path / 'zones.yaml'
    zones_yaml.write_text(
        'no_go_zones:\n'
        '  - name: block\n'
        '    type: polygon\n'
        '    points: [[0.5, 0.5], [1.0, 0.5], [1.0, 1.0], [0.5, 1.0]]\n',
        encoding='utf-8',
    )

    build(str(map_yaml), str(zones_yaml), str(tmp_path))

    assert raw_path.read_bytes() == raw_bytes
    with Image.open(tmp_path / 'no_go_mask.pgm') as mask:
        assert mask.convert('L').getpixel((7, 12)) == 0
        assert mask.convert('L').getpixel((0, 0)) == 255
    with Image.open(tmp_path / 'inno_map_nav.pgm') as nav:
        assert nav.convert('L').getpixel((7, 12)) == 0
        assert nav.convert('L').getpixel((0, 0)) == 254
    assert yaml.safe_load((tmp_path / 'no_go_mask.yaml').read_text())['image'] == 'no_go_mask.pgm'
    assert yaml.safe_load((tmp_path / 'inno_map_nav.yaml').read_text())['image'] == 'inno_map_nav.pgm'
