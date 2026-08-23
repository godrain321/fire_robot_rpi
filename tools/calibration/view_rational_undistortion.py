#!/usr/bin/env python3
"""View and save an original/Rational-undistorted image comparison."""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import os
from pathlib import Path
import sys
from typing import Any, Mapping, Sequence

import cv2
import numpy as np
import yaml


PROJECT_ROOT = Path(__file__).resolve().parents[2]
COEFFICIENT_ORDER = ("k1", "k2", "p1", "p2", "k3", "k4", "k5", "k6")


@dataclass(frozen=True)
class RationalCameraInfo:
    """Validated Rational Polynomial camera parameters."""

    camera_name: str
    image_size: tuple[int, int]
    camera_matrix: np.ndarray
    distortion_coefficients: np.ndarray


def _camera_info_matrix(
    payload: Mapping[str, Any], field: str, shape: tuple[int, int]
) -> np.ndarray:
    entry = payload.get(field)
    if not isinstance(entry, Mapping):
        raise ValueError(f"camera_info.yaml is missing mapping '{field}'")
    if entry.get("rows") != shape[0] or entry.get("cols") != shape[1]:
        raise ValueError(f"{field} must declare {shape[0]} rows and {shape[1]} cols")
    expected = shape[0] * shape[1]
    values = np.asarray(entry.get("data", []), dtype=np.float64)
    if values.size != expected or not np.all(np.isfinite(values)):
        raise ValueError(
            f"{field}.data must contain exactly {expected} finite values"
        )
    return values.reshape(shape).copy()


def load_rational_camera_info(path: Path | str) -> RationalCameraInfo:
    """Load CameraInfo and require the fixed eight-term Rational model."""

    source = Path(path).expanduser().resolve()
    if not source.is_file():
        raise FileNotFoundError(f"calibration file does not exist: {source}")
    payload = yaml.safe_load(source.read_text(encoding="utf-8"))
    if not isinstance(payload, Mapping):
        raise ValueError("camera_info.yaml root must be a mapping")
    if payload.get("distortion_model") != "rational_polynomial":
        raise ValueError("distortion_model must be 'rational_polynomial'")

    width = int(payload.get("image_width", 0))
    height = int(payload.get("image_height", 0))
    if width <= 0 or height <= 0:
        raise ValueError("image_width and image_height must be positive")
    matrix = _camera_info_matrix(payload, "camera_matrix", (3, 3))
    if matrix[0, 0] <= 0 or matrix[1, 1] <= 0:
        raise ValueError("camera_matrix focal lengths fx and fy must be positive")
    distortion = _camera_info_matrix(
        payload, "distortion_coefficients", (1, 8)
    ).reshape(8)
    return RationalCameraInfo(
        str(payload.get("camera_name", "camera")),
        (width, height),
        matrix,
        distortion,
    )


def _read_image(path: Path) -> np.ndarray:
    if not path.is_file():
        raise FileNotFoundError(f"source image does not exist: {path}")
    image = cv2.imdecode(np.fromfile(path, dtype=np.uint8), cv2.IMREAD_COLOR)
    if image is None:
        raise ValueError(f"OpenCV could not decode source image: {path}")
    return image


def _write_png(path: Path, image: np.ndarray) -> Path:
    path.parent.mkdir(parents=True, exist_ok=True)
    success, encoded = cv2.imencode(".png", image)
    if not success:
        raise OSError(f"OpenCV could not encode PNG: {path}")
    encoded.tofile(path)
    return path.resolve()


def _candidate_path(value: str, project_root: Path) -> Path:
    candidate = Path(value).expanduser()
    if candidate.is_absolute():
        return candidate
    from_cwd = (Path.cwd() / candidate).resolve()
    return from_cwd if from_cwd.is_file() else (project_root / candidate).resolve()


def select_source_image(
    image: Path | None,
    accepted_views_path: Path,
    project_root: Path = PROJECT_ROOT,
) -> Path:
    """Choose an explicit image, then the first existing accepted view."""

    if image is not None:
        selected = _candidate_path(str(image), project_root)
        if not selected.is_file():
            raise FileNotFoundError(f"source image does not exist: {selected}")
        return selected
    if accepted_views_path.is_file():
        for line in accepted_views_path.read_text(encoding="utf-8").splitlines():
            if line.strip():
                candidate = _candidate_path(line.strip(), project_root)
                if candidate.is_file():
                    return candidate
    fallback = sorted((project_root / "data" / "intrinsic").glob("*.png"))
    if fallback:
        return fallback[0].resolve()
    raise FileNotFoundError(
        "no source image was found in accepted_views.txt or data/intrinsic/*.png"
    )


def undistort_rational(
    image: np.ndarray, camera_info: RationalCameraInfo, alpha: float
) -> tuple[np.ndarray, np.ndarray, tuple[int, int, int, int]]:
    """Apply the validated eight Rational coefficients without resizing."""

    if not 0.0 <= alpha <= 1.0:
        raise ValueError("alpha must be between 0 and 1")
    width, height = camera_info.image_size
    if image.ndim != 3 or image.shape[2] != 3:
        raise ValueError("source image must be a three-channel color image")
    if (image.shape[1], image.shape[0]) != (width, height):
        raise ValueError(
            f"source resolution {image.shape[1]}x{image.shape[0]} differs from "
            f"calibration resolution {width}x{height}; resizing is not allowed"
        )

    matrix = camera_info.camera_matrix.copy()
    distortion = camera_info.distortion_coefficients.reshape(-1, 1).copy()
    new_matrix, roi = cv2.getOptimalNewCameraMatrix(
        matrix, distortion, (width, height), alpha, (width, height)
    )
    map_x, map_y = cv2.initUndistortRectifyMap(
        matrix,
        distortion,
        None,
        new_matrix,
        (width, height),
        cv2.CV_32FC1,
    )
    corrected = cv2.remap(
        image,
        map_x,
        map_y,
        cv2.INTER_LINEAR,
        borderMode=cv2.BORDER_CONSTANT,
    )
    return corrected, np.asarray(new_matrix), tuple(int(value) for value in roi)


def make_side_by_side(
    original: np.ndarray, corrected: np.ndarray, alpha: float
) -> np.ndarray:
    """Create one labelled, full-resolution side-by-side PNG."""

    if original.shape != corrected.shape:
        raise ValueError("original and corrected images must have the same shape")
    height, width = original.shape[:2]
    label_height = max(52, int(round(height * 0.065)))
    canvas = np.full((height + label_height, width * 2, 3), 28, dtype=np.uint8)
    canvas[label_height:, :width] = original
    canvas[label_height:, width:] = corrected
    cv2.line(
        canvas, (width, 0), (width, height + label_height), (235, 235, 235), 2
    )
    scale = max(0.62, min(1.15, width / 1280.0))
    text_y = int(label_height * 0.67)
    cv2.putText(
        canvas,
        "ORIGINAL (DISTORTED)",
        (18, text_y),
        cv2.FONT_HERSHEY_SIMPLEX,
        scale,
        (255, 255, 255),
        2,
        cv2.LINE_AA,
    )
    cv2.putText(
        canvas,
        f"UNDISTORTED - RATIONAL 8 (alpha={alpha:.2f})",
        (width + 18, text_y),
        cv2.FONT_HERSHEY_SIMPLEX,
        scale,
        (255, 255, 255),
        2,
        cv2.LINE_AA,
    )
    return canvas


def _display_comparison(comparison: np.ndarray, max_width: int) -> bool:
    if not (os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY")):
        return False
    shown = comparison
    if comparison.shape[1] > max_width:
        ratio = max_width / comparison.shape[1]
        shown = cv2.resize(
            comparison,
            (max_width, max(1, int(round(comparison.shape[0] * ratio)))),
            interpolation=cv2.INTER_AREA,
        )
    try:
        cv2.imshow("Camera calibration: original | Rational 8 undistorted", shown)
        print("비교 창에서 아무 키나 누르면 닫힙니다.")
        cv2.waitKey(0)
        cv2.destroyAllWindows()
        return True
    except cv2.error as exc:
        print(
            f"경고: GUI 창을 열지 못했습니다. 저장된 비교 PNG를 사용하세요: {exc}",
            file=sys.stderr,
        )
        return False


def build_argument_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Show and save an original | Rational 8 undistorted comparison."
    )
    parser.add_argument(
        "--calibration",
        type=Path,
        default=PROJECT_ROOT / "outputs/pi_camera3_wide_intrinsic/camera_info.yaml",
    )
    parser.add_argument("--image", type=Path, default=None)
    parser.add_argument("--accepted-views", type=Path, default=None)
    parser.add_argument("--output-dir", type=Path, default=None)
    parser.add_argument(
        "--alpha",
        type=float,
        default=0.0,
        help="0 crops to valid pixels; 1 preserves maximum field of view",
    )
    parser.add_argument("--no-display", action="store_true")
    parser.add_argument("--display-max-width", type=int, default=1920)
    return parser


def run(args: argparse.Namespace) -> dict[str, Path]:
    calibration_path = args.calibration.expanduser().resolve()
    camera_info = load_rational_camera_info(calibration_path)
    accepted_views = (
        args.accepted_views.expanduser().resolve()
        if args.accepted_views is not None
        else calibration_path.parent / "accepted_views.txt"
    )
    source = select_source_image(args.image, accepted_views)
    original = _read_image(source)
    corrected, _, _ = undistort_rational(original, camera_info, args.alpha)
    comparison = make_side_by_side(original, corrected, args.alpha)
    output_dir = (
        args.output_dir.expanduser().resolve()
        if args.output_dir is not None
        else calibration_path.parent / "comparison"
    )
    alpha_token = f"{args.alpha:.2f}".replace(".", "p")
    corrected_path = _write_png(
        output_dir / f"undistorted_{source.stem}_alpha_{alpha_token}.png",
        corrected,
    )
    comparison_path = _write_png(
        output_dir
        / f"original_vs_undistorted_{source.stem}_alpha_{alpha_token}.png",
        comparison,
    )

    print(f"원본 이미지: {source.resolve()}")
    print(f"왜곡 보정 이미지: {corrected_path}")
    print(f"동시 비교 이미지: {comparison_path}")
    print(f"보정계수 파일: {calibration_path}")
    print("왜곡계수 순서: [" + ", ".join(COEFFICIENT_ORDER) + "]")
    if not args.no_display and not _display_comparison(
        comparison, args.display_max_width
    ):
        print(
            "GUI 세션이 없어 비교 PNG만 저장했습니다. "
            "위 '동시 비교 이미지'를 여세요."
        )
    return {
        "original": source.resolve(),
        "corrected": corrected_path,
        "comparison": comparison_path,
        "calibration": calibration_path,
    }


def main(argv: Sequence[str] | None = None) -> int:
    parser = build_argument_parser()
    args = parser.parse_args(argv)
    if args.display_max_width <= 0:
        parser.error("--display-max-width must be positive")
    try:
        run(args)
    except (FileNotFoundError, OSError, ValueError, yaml.YAMLError) as exc:
        parser.exit(2, f"ERROR: {exc}\n")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
