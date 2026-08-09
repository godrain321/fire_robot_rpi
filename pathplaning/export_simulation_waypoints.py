#!/usr/bin/env python3
"""Convert a factory_v3 world trajectory into the ROS waypoint queue format.

This tool never imports or commands ROS.  It validates the transformed path
against the selected ROS occupancy map and only then writes a PoseArray-style
YAML document accepted by ``inno_autonav/waypoint_queue.py``.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import math
from pathlib import Path
from typing import Iterable, Mapping, Sequence

import numpy as np
from PIL import Image
import yaml


Point = tuple[float, float]
Cell = tuple[int, int]


class ExportError(ValueError):
    """Raised when conversion cannot be proven safe."""


@dataclass(frozen=True)
class Transform2D:
    source_frame: str
    target_frame: str
    translation_x_m: float
    translation_y_m: float
    rotation_deg: float
    flip_x: bool = False
    flip_y: bool = False

    def apply(self, point: Point) -> Point:
        x, y = _finite_point(point, "source point")
        x = -x if self.flip_x else x
        y = -y if self.flip_y else y
        angle = math.radians(self.rotation_deg)
        cosine, sine = math.cos(angle), math.sin(angle)
        return (
            cosine * x - sine * y + self.translation_x_m,
            sine * x + cosine * y + self.translation_y_m,
        )


@dataclass(frozen=True)
class OccupancyMap:
    resolution: float
    origin: tuple[float, float, float]
    width: int
    height: int
    occupancy: np.ndarray  # ROS order: row zero is the map's lower edge.

    def world_to_grid(self, point: Point) -> Cell:
        x, y = _finite_point(point, "map point")
        dx, dy = x - self.origin[0], y - self.origin[1]
        cosine, sine = math.cos(self.origin[2]), math.sin(self.origin[2])
        local_x = cosine * dx + sine * dy
        local_y = -sine * dx + cosine * dy
        return (
            int(math.floor(local_x / self.resolution)),
            int(math.floor(local_y / self.resolution)),
        )

    def check_cell(self, cell: Cell, allow_unknown: bool) -> None:
        x, y = cell
        if not (0 <= x < self.width and 0 <= y < self.height):
            raise ExportError(f"waypoint/segment lies outside map at grid {cell}")
        value = int(self.occupancy[y, x])
        if value >= 100:
            raise ExportError(f"waypoint/segment intersects occupied grid {cell}")
        if value < 0 and not allow_unknown:
            raise ExportError(f"waypoint/segment intersects unknown grid {cell}")


def _finite(value: object, label: str) -> float:
    try:
        result = float(value)
    except (TypeError, ValueError) as exc:
        raise ExportError(f"{label} must be numeric") from exc
    if not math.isfinite(result):
        raise ExportError(f"{label} must be finite")
    return result


def _finite_point(point: Sequence[object], label: str) -> Point:
    if not isinstance(point, Sequence) or isinstance(point, (str, bytes)) or len(point) != 2:
        raise ExportError(f"{label} must contain exactly x and y")
    return _finite(point[0], f"{label}.x"), _finite(point[1], f"{label}.y")


def _read_yaml(path: Path) -> Mapping:
    if not path.is_file():
        raise ExportError(f"file does not exist: {path}")
    try:
        document = yaml.safe_load(path.read_text(encoding="utf-8"))
    except (OSError, yaml.YAMLError) as exc:
        raise ExportError(f"cannot read YAML {path}: {exc}") from exc
    if not isinstance(document, Mapping):
        raise ExportError(f"YAML root must be a mapping: {path}")
    return document


def load_simulation_path(path: Path) -> tuple[str, tuple[Point, ...]]:
    document = _read_yaml(path)
    frame = document.get("coordinate_frame") or document.get("frame_id")
    if not isinstance(frame, str) or not frame.strip():
        raise ExportError("simulation path is missing coordinate_frame/frame_id")
    raw = document.get("points", document.get("path"))
    if not isinstance(raw, Sequence) or isinstance(raw, (str, bytes)):
        raise ExportError("simulation path needs a points/path sequence")
    points: list[Point] = []
    for index, entry in enumerate(raw):
        if isinstance(entry, Mapping):
            if "x" not in entry or "y" not in entry:
                raise ExportError(f"point {index} is missing x or y")
            candidate = (entry["x"], entry["y"])
        else:
            candidate = entry
        points.append(_finite_point(candidate, f"point {index}"))
    if len(points) < 2:
        raise ExportError("at least two path points are required to derive yaw")
    return frame, tuple(points)


def load_transform(path: Path) -> tuple[Transform2D, Mapping]:
    document = _read_yaml(path)
    raw = document.get("coordinate_transform")
    if not isinstance(raw, Mapping):
        raise ExportError("transform YAML needs coordinate_transform")
    required = ("source_frame", "target_frame", "translation_x_m",
                "translation_y_m", "rotation_deg", "flip_x", "flip_y")
    missing = [name for name in required if name not in raw]
    if missing:
        raise ExportError(f"transform fields missing: {', '.join(missing)}")
    if type(raw["flip_x"]) is not bool or type(raw["flip_y"]) is not bool:
        raise ExportError("flip_x and flip_y must be booleans")
    transform = Transform2D(
        str(raw["source_frame"]), str(raw["target_frame"]),
        _finite(raw["translation_x_m"], "translation_x_m"),
        _finite(raw["translation_y_m"], "translation_y_m"),
        _finite(raw["rotation_deg"], "rotation_deg"),
        raw["flip_x"], raw["flip_y"],
    )
    if not transform.source_frame or not transform.target_frame:
        raise ExportError("source_frame and target_frame cannot be empty")
    processing = document.get("waypoint_processing", {})
    if not isinstance(processing, Mapping):
        raise ExportError("waypoint_processing must be a mapping")
    return transform, processing


def load_occupancy_map(path: Path) -> OccupancyMap:
    document = _read_yaml(path)
    required = ("image", "resolution", "origin", "negate",
                "occupied_thresh", "free_thresh")
    missing = [name for name in required if name not in document]
    if missing:
        raise ExportError(f"map YAML fields missing: {', '.join(missing)}")
    resolution = _finite(document["resolution"], "map resolution")
    if resolution <= 0.0:
        raise ExportError("map resolution must be positive")
    origin_raw = document["origin"]
    if not isinstance(origin_raw, Sequence) or len(origin_raw) != 3:
        raise ExportError("map origin must be [x, y, yaw]")
    origin = tuple(_finite(value, f"origin[{index}]") for index, value in enumerate(origin_raw))
    negate = document["negate"]
    if type(negate) is not int or negate not in (0, 1):
        raise ExportError("map negate must be 0 or 1")
    occupied = _finite(document["occupied_thresh"], "occupied_thresh")
    free = _finite(document["free_thresh"], "free_thresh")
    if not 0.0 <= free < occupied <= 1.0:
        raise ExportError("map thresholds must satisfy 0 <= free < occupied <= 1")
    image_path = Path(str(document["image"])).expanduser()
    if not image_path.is_absolute():
        image_path = path.parent / image_path
    if not image_path.is_file():
        raise ExportError(f"map image does not exist: {image_path}")
    try:
        with Image.open(image_path) as image:
            pixels = np.asarray(image.convert("L"), dtype=np.uint8)
    except OSError as exc:
        raise ExportError(f"cannot read map image {image_path}: {exc}") from exc
    pixels = np.flipud(pixels)
    probability = ((255.0 - pixels) if negate == 0 else pixels) / 255.0
    data = np.full(pixels.shape, -1, dtype=np.int8)
    data[probability > occupied] = 100
    data[probability < free] = 0
    height, width = data.shape
    return OccupancyMap(resolution, origin, width, height, data)


def remove_duplicate_points(points: Iterable[Point], tolerance: float = 1e-9) -> tuple[Point, ...]:
    result: list[Point] = []
    for point in points:
        point = _finite_point(point, "path point")
        if not result or math.dist(result[-1], point) > tolerance:
            result.append(point)
    return tuple(result)


def supercover_cells(start: Cell, end: Cell) -> tuple[Cell, ...]:
    """Conservative grid traversal including cells touched at corners."""
    x0, y0 = start
    x1, y1 = end
    dx, dy = x1 - x0, y1 - y0
    steps = max(abs(dx), abs(dy))
    if steps == 0:
        return (start,)
    # Half-cell sampling is conservative at this map resolution; for exact
    # corner crossings also include both orthogonal neighbours.
    samples = max(1, steps * 2)
    ordered: list[Cell] = []
    for index in range(samples + 1):
        t = index / samples
        cell = (int(math.floor(x0 + 0.5 + dx * t)),
                int(math.floor(y0 + 0.5 + dy * t)))
        if not ordered or ordered[-1] != cell:
            if ordered:
                px, py = ordered[-1]
                if cell[0] != px and cell[1] != py:
                    for neighbour in ((cell[0], py), (px, cell[1])):
                        if neighbour not in ordered:
                            ordered.append(neighbour)
            if cell not in ordered:
                ordered.append(cell)
    return tuple(ordered)


def segment_is_safe(start: Point, end: Point, grid: OccupancyMap,
                    allow_unknown: bool) -> bool:
    try:
        for cell in supercover_cells(grid.world_to_grid(start), grid.world_to_grid(end)):
            grid.check_cell(cell, allow_unknown)
    except ExportError:
        return False
    return True


def simplify_path(points: Sequence[Point], grid: OccupancyMap, *,
                  minimum_spacing_m: float, direction_change_deg: float,
                  allow_unknown: bool,
                  maximum_spacing_m: float | None = None) -> tuple[Point, ...]:
    if (
        minimum_spacing_m < 0.0
        or maximum_spacing_m is not None
        and maximum_spacing_m <= 0.0
        or maximum_spacing_m is not None
        and maximum_spacing_m < minimum_spacing_m
        or not 0.0 <= direction_change_deg <= 180.0
    ):
        raise ExportError("invalid waypoint simplification settings")
    points = remove_duplicate_points(points)
    if len(points) < 2:
        raise ExportError("path collapsed below two distinct points")
    # First retain only genuine direction changes.  The previous implementation
    # also emitted a waypoint every ``minimum_spacing_m`` on straight runs,
    # which turned a visually straight 18 m trajectory into 52 ROS goals.
    mandatory_indices = [0]
    for index in range(1, len(points) - 1):
        previous, current, following = points[index - 1:index + 2]
        incoming = math.atan2(current[1] - previous[1], current[0] - previous[0])
        outgoing = math.atan2(following[1] - current[1], following[0] - current[0])
        turn = abs(math.degrees(math.atan2(math.sin(outgoing - incoming),
                                          math.cos(outgoing - incoming))))
        if turn >= direction_change_deg:
            mandatory_indices.append(index)
    mandatory_indices.append(len(points) - 1)

    # A sequence of tiny per-sample turns can form a visible gradual bend even
    # though no individual turn exceeds the angular threshold. Bound the
    # source-trajectory arc length between retained points so that curvature is
    # represented without returning to the old 0.3 m goal density.
    if maximum_spacing_m is not None:
        sampled_indices = [0]
        arc_distance = 0.0
        for index in range(1, len(points)):
            step_distance = math.dist(points[index - 1], points[index])
            if (
                arc_distance + step_distance > maximum_spacing_m + 1e-12
                and index - 1 > sampled_indices[-1]
            ):
                sampled_indices.append(index - 1)
                arc_distance = step_distance
            else:
                arc_distance += step_distance
        sampled_indices.append(len(points) - 1)
        mandatory_indices = sorted(set(mandatory_indices + sampled_indices))

    candidates = [points[0]]
    anchor_index = 0
    for target_index in mandatory_indices[1:]:
        target = points[target_index]
        if segment_is_safe(candidates[-1], target, grid, allow_unknown):
            candidates.append(target)
            anchor_index = target_index
            continue

        # A corner-to-corner shortcut is unsafe. Recover the farthest safe
        # original sample deterministically, then try the target again. This
        # preserves the source route without restoring fixed-distance goals.
        while anchor_index < target_index:
            safe_index = None
            for candidate_index in range(target_index, anchor_index, -1):
                if segment_is_safe(
                    candidates[-1], points[candidate_index], grid, allow_unknown
                ):
                    safe_index = candidate_index
                    break
            if safe_index is None or safe_index == anchor_index:
                raise ExportError("original path contains an unsafe map segment")
            candidate = points[safe_index]
            # The spacing value now suppresses only redundant near-identical
            # recovery samples; start, corners and goal remain mandatory.
            if (
                safe_index != target_index
                and math.dist(candidates[-1], candidate) < minimum_spacing_m
            ):
                safe_index = anchor_index + 1
                candidate = points[safe_index]
                if not segment_is_safe(
                    candidates[-1], candidate, grid, allow_unknown
                ):
                    raise ExportError("original path contains an unsafe map segment")
            candidates.append(candidate)
            anchor_index = safe_index
    if len(candidates) < 2:
        raise ExportError("simplified path has fewer than two points")
    validate_path(candidates, grid, allow_unknown)
    return tuple(candidates)


def validate_path(points: Sequence[Point], grid: OccupancyMap,
                  allow_unknown: bool) -> None:
    if len(points) < 2:
        raise ExportError("path needs at least two waypoints")
    for index, point in enumerate(points):
        grid.check_cell(grid.world_to_grid(point), allow_unknown)
        if index and not segment_is_safe(points[index - 1], point, grid, allow_unknown):
            raise ExportError(f"segment {index - 1}->{index} is not map-safe")


def yaw_values(points: Sequence[Point], final_yaw: float | None = None) -> tuple[float, ...]:
    if len(points) < 2:
        raise ExportError("at least two points are needed for yaw")
    values = [math.atan2(points[i + 1][1] - points[i][1],
                         points[i + 1][0] - points[i][0])
              for i in range(len(points) - 1)]
    values.append(values[-1] if final_yaw is None else _finite(final_yaw, "final yaw"))
    return tuple(values)


def build_waypoint_document(points: Sequence[Point], frame_id: str = "map",
                            final_yaw: float | None = None) -> dict:
    yaws = yaw_values(points, final_yaw)
    poses = []
    for (x, y), yaw in zip(points, yaws):
        qz, qw = math.sin(yaw / 2.0), math.cos(yaw / 2.0)
        poses.append({
            "header": {"stamp": {"sec": 0, "nanosec": 0}, "frame_id": frame_id},
            "pose": {
                "position": {"x": round(x, 9), "y": round(y, 9), "z": 0.0},
                "orientation": {"x": 0.0, "y": 0.0,
                                "z": round(qz, 12), "w": round(qw, 12)},
            },
        })
    return {"header": {"stamp": {"sec": 0, "nanosec": 0},
                       "frame_id": frame_id}, "poses": poses}


def validate_waypoint_document(document: Mapping, expected_frame: str = "map") -> int:
    header = document.get("header")
    poses = document.get("poses")
    if not isinstance(header, Mapping) or header.get("frame_id") != expected_frame:
        raise ExportError(f"waypoint header frame must be {expected_frame!r}")
    if not isinstance(poses, Sequence) or isinstance(poses, (str, bytes)) or not poses:
        raise ExportError("waypoint YAML needs a non-empty poses sequence")
    for index, entry in enumerate(poses):
        if not isinstance(entry, Mapping) or not isinstance(entry.get("pose"), Mapping):
            raise ExportError(f"waypoint {index} is malformed")
        pose = entry["pose"]
        position, orientation = pose.get("position"), pose.get("orientation")
        if not isinstance(position, Mapping) or not isinstance(orientation, Mapping):
            raise ExportError(f"waypoint {index} position/orientation is malformed")
        for name in ("x", "y", "z"):
            _finite(position.get(name), f"waypoint {index} position.{name}")
        quaternion = [_finite(orientation.get(name), f"waypoint {index} orientation.{name}")
                      for name in ("x", "y", "z", "w")]
        norm = math.sqrt(sum(value * value for value in quaternion))
        if not math.isclose(norm, 1.0, abs_tol=1e-6):
            raise ExportError(f"waypoint {index} quaternion is not normalized")
    return len(poses)


def export(args: argparse.Namespace) -> dict:
    source_frame, source = load_simulation_path(args.input)
    transform, processing = load_transform(args.transform)
    if source_frame != transform.source_frame:
        raise ExportError(f"source frame {source_frame!r} does not match transform "
                          f"{transform.source_frame!r}")
    if transform.target_frame != "map":
        raise ExportError("ROS waypoint target frame must be 'map'")
    grid = load_occupancy_map(args.map_yaml)
    transformed = tuple(transform.apply(point) for point in source)
    minimum_spacing = _finite(processing.get("minimum_spacing_m", 0.30),
                              "minimum_spacing_m")
    direction_change = _finite(processing.get("direction_change_deg", 8.0),
                               "direction_change_deg")
    maximum_spacing_raw = processing.get("maximum_spacing_m")
    maximum_spacing = (
        None if maximum_spacing_raw is None
        else _finite(maximum_spacing_raw, "maximum_spacing_m")
    )
    allow_unknown = processing.get("allow_unknown_cells", False)
    if type(allow_unknown) is not bool:
        raise ExportError("allow_unknown_cells must be boolean")
    simplified = simplify_path(
        transformed, grid, minimum_spacing_m=minimum_spacing,
        maximum_spacing_m=maximum_spacing,
        direction_change_deg=direction_change, allow_unknown=allow_unknown,
    )
    document = build_waypoint_document(simplified, transform.target_frame,
                                       args.final_yaw)
    validate_waypoint_document(document, transform.target_frame)
    if args.output.resolve(strict=False) == args.input.resolve(strict=False) and not args.overwrite:
        raise ExportError("input and output are identical; --overwrite is required")
    if args.output.exists() and not args.overwrite:
        raise ExportError(f"output exists; use --overwrite: {args.output}")
    if not args.dry_run:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(yaml.safe_dump(document, sort_keys=False), encoding="utf-8")
        validate_waypoint_document(_read_yaml(args.output), transform.target_frame)
    source_length = sum(math.dist(a, b) for a, b in zip(source, source[1:]))
    target_length = sum(math.dist(a, b) for a, b in zip(simplified, simplified[1:]))
    return {"source_points": len(source), "waypoints": len(simplified),
            "source_length_m": source_length, "waypoint_length_m": target_length,
            "written": not args.dry_run, "output": str(args.output)}


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", type=Path, required=True)
    parser.add_argument("--map-yaml", type=Path, required=True)
    parser.add_argument("--transform", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--final-yaw", type=float, default=None,
                        help="optional final yaw in radians")
    parser.add_argument("--overwrite", action="store_true")
    parser.add_argument("--dry-run", action="store_true",
                        help="validate and summarize without writing or using ROS")
    return parser.parse_args(argv)


def main(argv: Sequence[str] | None = None) -> int:
    try:
        summary = export(parse_args(argv))
    except (ExportError, OSError) as exc:
        print(f"ERROR: {exc}")
        return 2
    print(f"Source points: {summary['source_points']}")
    print(f"ROS waypoints: {summary['waypoints']}")
    print(f"Length: {summary['source_length_m']:.3f} m -> "
          f"{summary['waypoint_length_m']:.3f} m")
    print("Validated only (no ROS goals sent)." if not summary["written"]
          else f"Wrote: {summary['output']} (no ROS goals sent)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
