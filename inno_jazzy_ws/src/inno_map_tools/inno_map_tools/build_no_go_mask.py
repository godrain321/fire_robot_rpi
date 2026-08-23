"""Build a binary no-go mask and a planning map without modifying the raw map."""

import argparse
from pathlib import Path
import sys
from typing import Sequence

from PIL import Image, ImageChops, ImageDraw

from .map_utils import (
    MapToolsError,
    atomic_save_pgm,
    atomic_save_yaml,
    derived_map_metadata,
    load_map,
    load_zones,
    map_to_pixel,
)
from .project_paths import project_path


DEFAULT_MAP = project_path('maps', 'inno_map_raw.yaml')
DEFAULT_ZONES = project_path('maps', 'no_go_zones.yaml')
DEFAULT_OUT_DIR = project_path('maps')


def argument_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            '원본 occupancy map을 보존하면서 no-go mask와 planning map을 생성합니다.'
        )
    )
    parser.add_argument('--map-yaml', default=DEFAULT_MAP, help='원본 map YAML 경로')
    parser.add_argument('--zones-yaml', default=DEFAULT_ZONES, help='no-go zone YAML 경로')
    parser.add_argument('--out-dir', default=DEFAULT_OUT_DIR, help='출력 디렉터리')
    return parser


def build(map_yaml: str, zones_yaml: str, out_dir: str) -> Sequence[Path]:
    map_yaml_path, metadata, source_image_path, raw_map = load_map(map_yaml)
    _, zones = load_zones(zones_yaml)
    output_dir = Path(out_dir).expanduser().resolve(strict=False)
    if not output_dir.is_dir():
        raise MapToolsError(f'출력 디렉터리가 없습니다: {output_dir}')

    output_paths = {
        'mask_pgm': output_dir / 'no_go_mask.pgm',
        'mask_yaml': output_dir / 'no_go_mask.yaml',
        'nav_pgm': output_dir / 'inno_map_nav.pgm',
        'nav_yaml': output_dir / 'inno_map_nav.yaml',
    }
    protected = {map_yaml_path.resolve(), source_image_path.resolve()}
    for path in output_paths.values():
        if path.resolve() in protected:
            raise MapToolsError(f'출력이 원본 지도를 덮어쓸 수 없습니다: {path}')

    width, height = raw_map.size
    resolution = float(metadata['resolution'])
    origin_x = float(metadata['origin'][0])
    origin_y = float(metadata['origin'][1])
    mask = Image.new('L', (width, height), 255)
    draw = ImageDraw.Draw(mask)

    for zone in zones:
        pixels = []
        outside = []
        for map_x, map_y in zone['points']:
            pixel = map_to_pixel(
                map_x, map_y, origin_x, origin_y, resolution, height
            )
            pixels.append(pixel)
            if not (0 <= pixel[0] < width and 0 <= pixel[1] < height):
                outside.append((map_x, map_y, pixel[0], pixel[1]))
        if outside:
            print(
                f'경고: zone {zone["name"]!r}의 {len(outside)}개 꼭짓점이 '
                '지도 밖입니다. PIL clipping으로 가능한 범위만 처리합니다.',
                file=sys.stderr,
            )
            for map_x, map_y, pixel_x, pixel_y in outside:
                print(
                    f'  map=({map_x:.3f}, {map_y:.3f}) '
                    f'pixel=({pixel_x}, {pixel_y})',
                    file=sys.stderr,
                )
        draw.polygon(pixels, fill=0)

    planning_map = raw_map.copy()
    planning_map.paste(0, mask=ImageChops.invert(mask))

    atomic_save_pgm(output_paths['mask_pgm'], mask)
    atomic_save_yaml(
        output_paths['mask_yaml'],
        derived_map_metadata(metadata, output_paths['mask_pgm'].name),
    )
    atomic_save_pgm(output_paths['nav_pgm'], planning_map)
    atomic_save_yaml(
        output_paths['nav_yaml'],
        derived_map_metadata(metadata, output_paths['nav_pgm'].name),
    )
    print(f'적용된 no-go zone: {len(zones)}개')
    for label, path in output_paths.items():
        print(f'{label}: {path}')
    print(f'원본 지도 보존: {map_yaml_path}, {source_image_path}')
    return tuple(output_paths.values())


def main(args=None) -> None:
    parsed = argument_parser().parse_args(args)
    try:
        build(parsed.map_yaml, parsed.zones_yaml, parsed.out_dir)
    except MapToolsError as exc:
        print(f'오류: {exc}', file=sys.stderr)
        raise SystemExit(2) from exc


if __name__ == '__main__':
    main()
