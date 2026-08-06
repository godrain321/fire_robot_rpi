#!/usr/bin/env python3
"""Run offline checkerboard calibration with one fixed Rational model."""

from __future__ import annotations

import argparse
import csv
from dataclasses import replace
from datetime import datetime, timezone
import json
import logging
import math
from pathlib import Path
import shutil
import sys
from typing import Any, Iterable, Mapping, Sequence

import cv2
import numpy as np
import yaml

# Support both ``python -m tools.calibration...`` and direct script execution.
if __package__ in (None, ""):
    sys.path.insert(0, str(Path(__file__).resolve().parent))
    from calibration_core import (  # type: ignore
        CalibrationConfig,
        CalibrationResult,
        CheckerboardConfig,
        DetectedView,
        POSE_DESCRIPTOR_NAMES,
        RATIONAL_CALIBRATION_FLAGS,
        calibrate_rational,
        compute_coverage,
        coverage_warnings,
        create_object_points,
        detect_checkerboards,
        discover_images,
        filter_accepted_views,
        load_config,
        mark_pose_duplicates,
        pose_diverse_split,
        reject_outlier_views,
        remove_marked_duplicates,
        write_camera_info_yaml,
    )
    from calibration_validation import (  # type: ignore
        ValidationResult,
        check_rational_stability,
        cross_validate,
        validate_fixed_intrinsics,
        write_cross_validation_outputs,
        write_rational_stability_outputs,
        write_validation_outputs,
    )
    from calibration_visualization import (  # type: ignore
        save_coverage_heatmap,
        save_detection_overlay,
        save_undistorted_samples,
    )
else:
    from .calibration_core import (
        CalibrationConfig,
        CalibrationResult,
        CheckerboardConfig,
        DetectedView,
        POSE_DESCRIPTOR_NAMES,
        RATIONAL_CALIBRATION_FLAGS,
        calibrate_rational,
        compute_coverage,
        coverage_warnings,
        create_object_points,
        detect_checkerboards,
        discover_images,
        filter_accepted_views,
        load_config,
        mark_pose_duplicates,
        pose_diverse_split,
        reject_outlier_views,
        remove_marked_duplicates,
        write_camera_info_yaml,
    )
    from .calibration_validation import (
        ValidationResult,
        check_rational_stability,
        cross_validate,
        validate_fixed_intrinsics,
        write_cross_validation_outputs,
        write_rational_stability_outputs,
        write_validation_outputs,
    )
    from .calibration_visualization import (
        save_coverage_heatmap,
        save_detection_overlay,
        save_undistorted_samples,
    )


LOGGER = logging.getLogger("checkerboard_rational_calibration")


def build_argument_parser() -> argparse.ArgumentParser:
    """Build the command-line interface without parsing process arguments."""

    parser = argparse.ArgumentParser(
        description=(
            "Offline checkerboard calibration using only OpenCV's Rational "
            "Polynomial model D=[k1,k2,p1,p2,k3,k4,k5,k6]."
        )
    )
    parser.add_argument("--images", required=True, help="Input image glob pattern")
    parser.add_argument("--config", type=Path, default=None, help="YAML configuration")
    parser.add_argument("--output-dir", required=True, type=Path)
    parser.add_argument("--board-cols", type=int, default=None)
    parser.add_argument("--board-rows", type=int, default=None)
    parser.add_argument("--square-size-m", type=float, default=None)
    parser.add_argument("--validation-ratio", type=float, default=None)
    parser.add_argument("--cv-folds", type=int, default=None)
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--camera-name", default="pi_camera3_wide")
    parser.add_argument(
        "--strict-resolution",
        action="store_true",
        default=None,
        help="Fail instead of excluding an image with a different resolution",
    )
    parser.add_argument(
        "--remove-duplicates",
        action="store_true",
        default=None,
        help="Exclude descriptor-marked duplicate poses",
    )
    parser.add_argument("--force", action="store_true")
    parser.add_argument(
        "--log-level",
        choices=("DEBUG", "INFO", "WARNING", "ERROR"),
        default="INFO",
    )
    return parser


def _load_runtime_options(path: Path | None) -> dict[str, Any]:
    if path is None:
        return {}
    payload = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
    calibration = payload.get("calibration", {}) if isinstance(payload, Mapping) else {}
    return dict(calibration) if isinstance(calibration, Mapping) else {}


def _merged_configuration(
    args: argparse.Namespace,
) -> tuple[CheckerboardConfig, CalibrationConfig, dict[str, Any]]:
    board, calibration = load_config(args.config)
    board = replace(
        board,
        inner_corners_cols=(
            args.board_cols
            if args.board_cols is not None
            else board.inner_corners_cols
        ),
        inner_corners_rows=(
            args.board_rows
            if args.board_rows is not None
            else board.inner_corners_rows
        ),
        square_size_m=(
            args.square_size_m
            if args.square_size_m is not None
            else board.square_size_m
        ),
    )
    calibration = replace(
        calibration,
        validation_ratio=(
            args.validation_ratio
            if args.validation_ratio is not None
            else calibration.validation_ratio
        ),
        cv_folds=args.cv_folds if args.cv_folds is not None else calibration.cv_folds,
    )
    raw = _load_runtime_options(args.config)
    runtime = {
        "strict_resolution": (
            bool(args.strict_resolution)
            if args.strict_resolution is not None
            else bool(raw.get("strict_resolution", False))
        ),
        "remove_duplicates": (
            bool(args.remove_duplicates)
            if args.remove_duplicates is not None
            else bool(raw.get("remove_duplicates", False))
        ),
        "sample_undistort_count": int(raw.get("sample_undistort_count", 5)),
    }
    if runtime["sample_undistort_count"] < 0:
        raise ValueError("sample_undistort_count cannot be negative")
    return board, calibration, runtime


def _prepare_output_directory(path: Path, force: bool) -> Path:
    destination = path.expanduser().resolve()
    if destination.exists():
        if not destination.is_dir():
            raise FileExistsError(f"output path is not a directory: {destination}")
        if any(destination.iterdir()):
            if not force:
                raise FileExistsError(
                    f"output directory is not empty: {destination}; pass --force to replace it"
                )
            shutil.rmtree(destination)
    destination.mkdir(parents=True, exist_ok=True)
    for relative in (
        "detections/accepted",
        "detections/rejected",
        "validation_overlays",
        "undistorted_samples",
    ):
        (destination / relative).mkdir(parents=True, exist_ok=True)
    return destination


def _json_ready(value: Any) -> Any:
    if isinstance(value, np.ndarray):
        return _json_ready(value.tolist())
    if isinstance(value, np.generic):
        return _json_ready(value.item())
    if isinstance(value, Path):
        return str(value)
    if isinstance(value, Mapping):
        return {str(key): _json_ready(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_json_ready(item) for item in value]
    if isinstance(value, float) and not math.isfinite(value):
        return None
    return value


def _write_json(path: Path, payload: Any) -> None:
    path.write_text(
        json.dumps(_json_ready(payload), ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )


def _write_lines(path: Path, values: Iterable[str | Path]) -> None:
    path.write_text("".join(f"{value}\n" for value in values), encoding="utf-8")


def _write_csv(path: Path, rows: Sequence[Mapping[str, Any]], fields: Sequence[str]) -> None:
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(fields), extrasaction="ignore")
        writer.writeheader()
        for row in rows:
            writer.writerow({key: _json_ready(row.get(key, "")) for key in fields})


def _save_detection_outputs(
    views: Sequence[DetectedView], board: CheckerboardConfig, output_dir: Path
) -> None:
    metadata = [view.to_metadata_dict() for view in views]
    fields = (
        "path",
        "width",
        "height",
        "read_success",
        "detection_success",
        "corner_count",
        "center_x",
        "center_y",
        "area_ratio",
        "blur_score",
        "exclusion_reason",
        "duplicate_of",
        *POSE_DESCRIPTOR_NAMES,
    )
    _write_csv(output_dir / "image_metadata.csv", metadata, fields)
    for index, view in enumerate(views):
        group = "accepted" if view.accepted else "rejected"
        destination = (
            output_dir
            / "detections"
            / group
            / f"{index:04d}_{view.path.stem}.png"
        )
        save_detection_overlay(view, board, destination)


def _save_pose_and_coverage(
    views: Sequence[DetectedView],
    image_size: tuple[int, int],
    config: CalibrationConfig,
    output_dir: Path,
) -> tuple[np.ndarray, list[str]]:
    pose_rows = [view.to_metadata_dict() for view in views]
    _write_csv(
        output_dir / "pose_descriptors.csv",
        pose_rows,
        ("path", "duplicate_of", *POSE_DESCRIPTOR_NAMES),
    )
    counts = compute_coverage(
        views,
        image_size,
        config.coverage_grid_cols,
        config.coverage_grid_rows,
    )
    with (output_dir / "coverage_counts.csv").open(
        "w", newline="", encoding="utf-8"
    ) as stream:
        writer = csv.writer(stream)
        writer.writerow(["grid_row/grid_col", *range(counts.shape[1])])
        for row_index, row in enumerate(counts):
            writer.writerow([row_index, *[int(value) for value in row]])
    save_coverage_heatmap(counts, output_dir / "coverage_heatmap.png")
    return counts, coverage_warnings(counts)


def _write_split_outputs(
    training: Sequence[DetectedView],
    validation: Sequence[DetectedView],
    seed: int,
    ratio: float,
    output_dir: Path,
) -> None:
    _write_lines(output_dir / "training_views.txt", (view.path for view in training))
    _write_lines(output_dir / "validation_views.txt", (view.path for view in validation))
    _write_json(
        output_dir / "split_summary.json",
        {
            "method": "seeded_greedy_farthest_point_pose_groups",
            "seed": seed,
            "requested_validation_ratio": ratio,
            "training_view_count": len(training),
            "validation_view_count": len(validation),
            "actual_validation_ratio": len(validation) / max(1, len(training) + len(validation)),
            "training_views": [str(view.path) for view in training],
            "validation_views": [str(view.path) for view in validation],
        },
    )


def _write_rejections(
    all_views: Sequence[DetectedView],
    outlier_records: Sequence[Any],
    output_dir: Path,
    *,
    duplicate_views: Sequence[DetectedView] = (),
) -> None:
    rows: list[dict[str, Any]] = []
    for view in all_views:
        if not view.accepted:
            rows.append(
                {
                    "path": str(view.path),
                    "per_view_rms": "",
                    "robust_threshold": "",
                    "iteration": 0,
                    "coverage_impact": "",
                    "reason": view.exclusion_reason or "detection_rejected",
                }
            )
    rows.extend(
        {
            "path": str(view.path),
            "per_view_rms": "",
            "robust_threshold": "",
            "iteration": 0,
            "coverage_impact": "",
            "reason": "duplicate_pose_removed",
        }
        for view in duplicate_views
    )
    rows.extend(record.to_dict() for record in outlier_records)
    _write_csv(
        output_dir / "rejected_views.csv",
        rows,
        (
            "path",
            "per_view_rms",
            "robust_threshold",
            "iteration",
            "coverage_impact",
            "reason",
        ),
    )


def _write_per_view_errors(result: CalibrationResult, output_dir: Path) -> None:
    _write_csv(
        output_dir / "per_view_errors.csv",
        [
            {"path": str(view.path), "rms_px": float(error)}
            for view, error in zip(result.views, result.per_view_errors, strict=True)
        ],
        ("path", "rms_px"),
    )


def _quality_status(
    stability: Mapping[str, Any],
    validation: ValidationResult,
    cv_summary: Mapping[str, Any],
    warnings: Sequence[str],
) -> str:
    if not bool(stability.get("stable", False)):
        return "failed_quality_check"
    if (
        warnings
        or validation.warnings
        or cv_summary.get("quality_status") not in (None, "passed")
    ):
        return "warning"
    return "passed"


def _write_report(
    path: Path,
    board: CheckerboardConfig,
    image_size: tuple[int, int],
    development: CalibrationResult,
    final: CalibrationResult,
    validation: ValidationResult,
    quality_status: str,
    warnings: Sequence[str],
    counts: Mapping[str, int],
) -> None:
    d_text = ", ".join(f"{value:.12g}" for value in final.D)
    lines = [
        "# Checkerboard Rational Polynomial 캘리브레이션 보고서",
        "",
        f"- 품질 상태: `{quality_status}`",
        "- 모델: OpenCV Rational Polynomial 8계수 (단일 고정 모델)",
        f"- 해상도: {image_size[0]} × {image_size[1]}",
        f"- 체커보드: {board.cols} × {board.rows} 내부 코너",
        f"- 한 칸 길이: {board.square_size_m:.6f} m",
        f"- 검출 성공/입력: {counts['accepted']} / {counts['input']}",
        f"- 개발 학습 RMS: {development.rms:.6f} px",
        f"- 검증 RMS: {validation.validation_rms:.6f} px",
        f"- 최종 전체 재학습 RMS: {final.rms:.6f} px",
        "",
        "## 배포용 최종 파라미터",
        "",
        "```text",
        np.array2string(final.K, precision=12),
        f"D = [{d_text}]",
        "order = [k1, k2, p1, p2, k3, k4, k5, k6]",
        "```",
        "",
        "`camera_info.yaml`은 위 최종 전체 재학습 K/D를 사용한다.",
        "왜곡 보정 샘플의 new camera matrix는 배포용 원본 K를 대체하지 않는다.",
    ]
    if warnings:
        lines.extend(["", "## 경고", ""] + [f"- {warning}" for warning in warnings])
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def run(args: argparse.Namespace) -> Path:
    """Execute the full pipeline and return the result directory."""

    board, config, runtime = _merged_configuration(args)
    image_paths = discover_images(args.images)
    output_dir = _prepare_output_directory(args.output_dir, args.force)
    LOGGER.info("Found %d input images", len(image_paths))
    views, image_size = detect_checkerboards(
        image_paths, board, strict_resolution=runtime["strict_resolution"]
    )
    accepted = filter_accepted_views(views)
    if not accepted:
        raise RuntimeError("No complete checkerboard detections were accepted")

    mark_pose_duplicates(accepted, config.duplicate_distance_threshold)
    _save_detection_outputs(views, board, output_dir)
    duplicate_views = [
        view
        for view in accepted
        if runtime["remove_duplicates"] and view.duplicate_of is not None
    ]
    if runtime["remove_duplicates"]:
        accepted = remove_marked_duplicates(accepted)
    minimum_total = config.minimum_training_views + (
        1 if config.validation_ratio > 0.0 else 0
    )
    if len(accepted) < minimum_total:
        raise RuntimeError(
            f"Only {len(accepted)} usable views remain; at least {minimum_total} are "
            "needed to preserve minimum_training_views after validation splitting"
        )

    counts_grid, distribution_warnings = _save_pose_and_coverage(
        accepted, image_size, config, output_dir
    )
    training, validation_views = pose_diverse_split(
        accepted, config.validation_ratio, args.seed
    )
    if not validation_views:
        raise RuntimeError("validation_ratio produced no held-out validation views")
    if len(training) < config.minimum_training_views:
        raise RuntimeError(
            f"Training split has {len(training)} views; need "
            f"{config.minimum_training_views}. Capture more diverse images."
        )
    _write_split_outputs(
        training, validation_views, args.seed, config.validation_ratio, output_dir
    )

    development, _retained_training, outlier_records = reject_outlier_views(
        training, board, image_size, config
    )
    object_points = create_object_points(board)
    validation = validate_fixed_intrinsics(
        validation_views,
        object_points,
        development.K,
        development.D,
        image_size,
    )
    write_validation_outputs(
        validation, output_dir, views=validation_views, image_size=image_size
    )

    excluded_outlier_paths = {record.path for record in outlier_records}
    final_views = [
        view
        for view in accepted
        if view.path not in excluded_outlier_paths
        and (not runtime["remove_duplicates"] or view.duplicate_of is None)
    ]
    final_calibration = calibrate_rational(final_views, board, image_size, config)

    cv_config = {**config.to_dict(), "seed": int(args.seed)}
    cv_rows, cv_summary = cross_validate(
        final_views, object_points, image_size, cv_config
    )
    write_cross_validation_outputs(cv_rows, cv_summary, output_dir)
    stability = check_rational_stability(
        final_calibration.K, final_calibration.D, image_size
    )
    write_rational_stability_outputs(stability, output_dir)

    all_warnings = list(
        dict.fromkeys(
            distribution_warnings
            + development.warnings
            + final_calibration.warnings
            + validation.warnings
            + list(cv_summary.get("warnings", []))
            + list(stability.get("warnings", []))
            + list(stability.get("issues", []))
        )
    )
    status = _quality_status(stability, validation, cv_summary, all_warnings)
    _write_lines(output_dir / "accepted_views.txt", (view.path for view in final_views))
    _write_rejections(
        views,
        outlier_records,
        output_dir,
        duplicate_views=duplicate_views,
    )
    _write_per_view_errors(final_calibration, output_dir)
    write_camera_info_yaml(
        output_dir / "camera_info.yaml",
        args.camera_name,
        image_size,
        final_calibration,
    )
    sample_paths = [view.path for view in final_views[: runtime["sample_undistort_count"]]]
    save_undistorted_samples(
        sample_paths,
        final_calibration.K,
        final_calibration.D,
        image_size,
        output_dir / "undistorted_samples",
    )

    result_payload = {
        "image_width": image_size[0],
        "image_height": image_size[1],
        "camera_name": args.camera_name,
        "board": board.to_dict(),
        "calibration": config.to_dict(),
        "model": "opencv_rational_polynomial_8",
        "distortion_coefficient_order": [
            "k1", "k2", "p1", "p2", "k3", "k4", "k5", "k6"
        ],
        "calibration_flags": int(RATIONAL_CALIBRATION_FLAGS),
        "opencv_version": cv2.__version__,
        "timestamp_utc": datetime.now(timezone.utc).isoformat(),
        "development_split_calibration": development.to_dict(),
        "validation_metrics": validation.summary,
        "cross_validation_metrics": cv_summary,
        "final_all_valid_images_calibration": final_calibration.to_dict(),
        "K": final_calibration.K.tolist(),
        "D": final_calibration.D.tolist(),
        "overall_rms": final_calibration.rms,
        "per_view_rms": final_calibration.per_view_errors.tolist(),
        "intrinsic_standard_deviations": final_calibration.std_intrinsics.tolist(),
        "accepted_images": [str(view.path) for view in final_views],
        "rejected_images": [
            {
                "path": str(view.path),
                "reason": view.exclusion_reason or "detection_rejected",
            }
            for view in views
            if not view.accepted
        ] + [
            {"path": str(view.path), "reason": "duplicate_pose_removed"}
            for view in duplicate_views
        ] + [
            {"path": str(record.path), "reason": record.reason}
            for record in outlier_records
        ],
        "rational_stability": stability,
        "coverage_counts": counts_grid,
        "quality_status": status,
        "warnings": all_warnings,
    }
    _write_json(output_dir / "calibration_result.json", result_payload)
    _write_report(
        output_dir / "calibration_report.md",
        board,
        image_size,
        development,
        final_calibration,
        validation,
        status,
        all_warnings,
        {"input": len(views), "accepted": len(final_views)},
    )
    LOGGER.info("Final Rational RMS: %.6f px", final_calibration.rms)
    LOGGER.info("Quality status: %s", status)
    return output_dir


def main(argv: Sequence[str] | None = None) -> int:
    """CLI entry point with concise, actionable failures."""

    parser = build_argument_parser()
    args = parser.parse_args(argv)
    logging.basicConfig(
        level=getattr(logging, args.log_level),
        format="%(levelname)s: %(message)s",
    )
    try:
        output_dir = run(args)
    except (FileNotFoundError, FileExistsError, ValueError, RuntimeError) as exc:
        LOGGER.error("%s", exc)
        return 2
    print(f"Rational Polynomial 8-coefficient calibration saved to: {output_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
