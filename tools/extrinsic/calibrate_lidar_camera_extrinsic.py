#!/usr/bin/env python3
"""Command-line 2-D lidar/camera extrinsic calibrator."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys
from typing import Any

import yaml


PROJECT_ROOT = Path(__file__).resolve().parents[2]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from tools.extrinsic.extrinsic_core import (  # noqa: E402
    SolverConfig,
    calibrate_extrinsic,
    load_observations_json,
    load_rational_camera_info,
    result_payload,
)


YAML_NAME = "lidar_camera_extrinsic.yaml"
JSON_NAME = "extrinsic_calibration_result.json"
REPORT_NAME = "extrinsic_calibration_report.txt"


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Estimate p_camera = R_camera_lidar*p_lidar + t_camera_lidar_m "
            "from checkerboard plane/lidar point observations."
        )
    )
    parser.add_argument("--observations", type=Path, required=True)
    parser.add_argument("--camera-info", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--force", action="store_true", help="overwrite result files")
    parser.add_argument("--min-views", type=int, default=20)
    parser.add_argument("--huber-delta-m", type=float, default=0.015)
    parser.add_argument("--max-iterations", type=int, default=120)
    parser.add_argument("--max-condition-number", type=float, default=1.0e8)
    parser.add_argument("--max-rmse-m", type=float, default=0.020)
    parser.add_argument("--max-pose-rmse-m", type=float, default=0.030)
    return parser


def _report_text(payload: dict[str, Any], observations_path: Path) -> str:
    frames = payload["frames"]
    quality = payload["quality"]
    optimization = payload["optimization"]
    translation = payload["t_camera_lidar_m"]
    rotation = payload["R_camera_lidar"]
    return "\n".join(
        [
            "2-D LIDAR / CAMERA EXTRINSIC CALIBRATION REPORT",
            "================================================",
            f"Observations: {observations_path.resolve()}",
            f"CameraInfo: {payload['camera_intrinsics']['camera_info_path']}",
            f"Convention: {payload['transform_convention']}",
            f"Frames: {frames['lidar']} -> {frames['camera']}",
            f"Views / lidar points: {quality['pose_count']} / {quality['lidar_point_count']}",
            f"Converged / iterations: {optimization['converged']} / {optimization['iterations']}",
            f"Plane residual RMSE: {quality['rmse_m']:.9f} m",
            f"Allowed plane residual RMSE: {quality['max_allowed_rmse_m']:.9f} m",
            f"Median absolute residual: {quality['median_abs_residual_m']:.9f} m",
            f"Maximum absolute residual: {quality['max_abs_residual_m']:.9f} m",
            f"Allowed per-pose RMSE: {quality['max_allowed_pose_rmse_m']:.9f} m",
            f"Normal rank: {quality['normal_rank']} (required: 3)",
            f"Jacobian rank: {quality['jacobian_rank']} (required: 6)",
            f"Jacobian condition number: {quality['jacobian_condition_number']:.6g}",
            "Translation camera<-lidar [m]: " + " ".join(f"{v:.12g}" for v in translation),
            "Rotation camera<-lidar:",
            *("  " + " ".join(f"{v:.12g}" for v in row) for row in rotation),
            "",
        ]
    )


def _reserve_outputs(output_dir: Path, force: bool) -> tuple[Path, Path, Path]:
    output_dir = output_dir.expanduser().resolve()
    targets = (
        output_dir / YAML_NAME,
        output_dir / JSON_NAME,
        output_dir / REPORT_NAME,
    )
    existing = [path for path in targets if path.exists()]
    if existing and not force:
        names = ", ".join(str(path) for path in existing)
        raise FileExistsError(f"result files already exist (use --force): {names}")
    output_dir.mkdir(parents=True, exist_ok=True)
    return targets


def run(args: argparse.Namespace) -> tuple[Path, Path, Path]:
    config = SolverConfig(
        min_views=args.min_views,
        huber_delta_m=args.huber_delta_m,
        max_iterations=args.max_iterations,
        max_jacobian_condition_number=args.max_condition_number,
        max_rmse_m=args.max_rmse_m,
        max_pose_rmse_m=args.max_pose_rmse_m,
    )
    camera_info = load_rational_camera_info(args.camera_info)
    observation_set = load_observations_json(
        args.observations, min_views=config.min_views, camera_info=camera_info
    )
    result = calibrate_extrinsic(observation_set, config)
    payload = result_payload(result, observation_set, camera_info, config)
    yaml_path, json_path, report_path = _reserve_outputs(args.output_dir, args.force)
    yaml_path.write_text(
        yaml.safe_dump(payload, sort_keys=False, allow_unicode=True), encoding="utf-8"
    )
    detailed = dict(payload)
    detailed["observations_path"] = str(Path(args.observations).expanduser().resolve())
    detailed["per_pose_rmse_m"] = {
        observation.pose_id: rmse
        for observation, rmse in zip(observation_set.observations, result.pose_rmse_m)
    }
    detailed["raw_plane_residuals_m"] = result.raw_residuals_m.tolist()
    json_path.write_text(json.dumps(detailed, indent=2) + "\n", encoding="utf-8")
    report_path.write_text(_report_text(payload, Path(args.observations)), encoding="utf-8")
    return yaml_path, json_path, report_path


def main(argv: list[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    try:
        yaml_path, json_path, report_path = run(args)
    except (OSError, ValueError, RuntimeError, json.JSONDecodeError, yaml.YAMLError) as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 2
    print("Extrinsic calibration passed all geometry/conditioning checks.")
    print(f"Transform YAML: {yaml_path}")
    print(f"Detailed JSON: {json_path}")
    print(f"Text report: {report_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
