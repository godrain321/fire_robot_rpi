"""Offline OpenCV fisheye calibration from captured checkerboard images."""

import argparse
from pathlib import Path

import cv2
import numpy as np
import yaml


def _image_paths(image_directory):
    paths = []
    for extension in ('*.png', '*.jpg', '*.jpeg'):
        paths.extend(image_directory.glob(extension))
        paths.extend(image_directory.glob(extension.upper()))
    return sorted(set(paths))


def _detect_corners(gray_image, pattern_size):
    flags = cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE
    found, corners = cv2.findChessboardCorners(
        gray_image,
        pattern_size,
        flags,
    )
    if not found:
        return None

    criteria = (
        cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER,
        30,
        1e-3,
    )
    return cv2.cornerSubPix(
        gray_image,
        corners,
        winSize=(11, 11),
        zeroZone=(-1, -1),
        criteria=criteria,
    )


def _parser():
    parser = argparse.ArgumentParser(
        description='Calibrate an equidistant/fisheye camera from images.',
    )
    parser.add_argument('--image-dir', required=True, type=Path)
    parser.add_argument('--output-yaml', required=True, type=Path)
    parser.add_argument('--board-cols', type=int, default=8)
    parser.add_argument('--board-rows', type=int, default=9)
    parser.add_argument('--square-size', type=float, default=0.07)
    parser.add_argument('--camera-name', default='camera')
    parser.add_argument('--minimum-images', type=int, default=15)
    return parser


def main():
    """Run offline fisheye calibration and write camera_info YAML."""
    args = _parser().parse_args()

    if args.board_cols < 2 or args.board_rows < 2:
        raise ValueError('Checkerboard inner-corner counts must be at least 2')
    if args.square_size <= 0.0:
        raise ValueError('--square-size must be positive')

    pattern_size = (args.board_cols, args.board_rows)
    image_paths = _image_paths(args.image_dir)
    if not image_paths:
        raise RuntimeError(
            f'No PNG or JPEG images found in {args.image_dir}'
        )

    object_template = np.zeros(
        (1, args.board_cols * args.board_rows, 3),
        np.float64,
    )
    grid = np.mgrid[
        0:args.board_cols,
        0:args.board_rows,
    ].T.reshape(-1, 2)
    object_template[0, :, :2] = grid * args.square_size

    object_points = []
    image_points = []
    image_size = None
    failures = 0

    for image_path in image_paths:
        image = cv2.imread(str(image_path), cv2.IMREAD_COLOR)
        if image is None:
            failures += 1
            print(f'[FAIL] cannot read: {image_path}')
            continue

        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        current_size = gray.shape[::-1]
        if image_size is None:
            image_size = current_size
        elif image_size != current_size:
            raise RuntimeError(
                f'Image size mismatch: {image_path} is {current_size}, '
                f'expected {image_size}'
            )

        corners = _detect_corners(gray, pattern_size)
        if corners is None:
            failures += 1
            print(f'[FAIL] corners: {image_path}')
            continue

        object_points.append(object_template.copy())
        image_points.append(corners.astype(np.float64))
        print(f'[OK] {image_path}')

    used_images = len(image_points)
    if used_images < args.minimum_images:
        raise RuntimeError(
            f'Only {used_images} valid images. Need at least '
            f'{args.minimum_images}; 30 or more diverse images are recommended.'
        )

    camera_matrix = np.zeros((3, 3), dtype=np.float64)
    distortion = np.zeros((4, 1), dtype=np.float64)
    rotation_vectors = [
        np.zeros((1, 1, 3), dtype=np.float64)
        for _ in range(used_images)
    ]
    translation_vectors = [
        np.zeros((1, 1, 3), dtype=np.float64)
        for _ in range(used_images)
    ]
    flags = (
        cv2.fisheye.CALIB_RECOMPUTE_EXTRINSIC
        | cv2.fisheye.CALIB_FIX_SKEW
    )
    criteria = (
        cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER,
        100,
        1e-6,
    )

    rms, camera_matrix, distortion, _, _ = cv2.fisheye.calibrate(
        object_points,
        image_points,
        image_size,
        camera_matrix,
        distortion,
        rotation_vectors,
        translation_vectors,
        flags,
        criteria,
    )

    rectification = np.eye(3, dtype=np.float64)
    projection = np.zeros((3, 4), dtype=np.float64)
    projection[:, :3] = camera_matrix
    width, height = image_size

    output = {
        'image_width': int(width),
        'image_height': int(height),
        'camera_name': args.camera_name,
        'camera_matrix': {
            'rows': 3,
            'cols': 3,
            'data': camera_matrix.reshape(-1).tolist(),
        },
        'distortion_model': 'equidistant',
        'distortion_coefficients': {
            'rows': 1,
            'cols': 4,
            'data': distortion.reshape(-1).tolist(),
        },
        'rectification_matrix': {
            'rows': 3,
            'cols': 3,
            'data': rectification.reshape(-1).tolist(),
        },
        'projection_matrix': {
            'rows': 3,
            'cols': 4,
            'data': projection.reshape(-1).tolist(),
        },
        'calibration_info': {
            'model': 'opencv_fisheye',
            'board_inner_corners': [
                args.board_cols,
                args.board_rows,
            ],
            'square_size_m': args.square_size,
            'used_images': used_images,
            'failed_images': failures,
            'rms_reprojection_error_px': float(rms),
        },
    }

    args.output_yaml.parent.mkdir(parents=True, exist_ok=True)
    with args.output_yaml.open('w', encoding='utf-8') as stream:
        yaml.safe_dump(output, stream, sort_keys=False)

    print('\n========== FISHEYE CALIBRATION RESULT ==========')
    print(f'image size: {width} x {height}')
    print(f'used / failed images: {used_images} / {failures}')
    print(f'RMS reprojection error: {rms:.4f} px')
    print(f'K:\n{camera_matrix}')
    print(f'D: {distortion.reshape(-1)}')
    print(f'Saved: {args.output_yaml}')
    if rms < 1.0:
        print('Assessment: good')
    elif rms < 2.0:
        print('Assessment: usable; verify rectification before extrinsics')
    else:
        print('Assessment: recapture with more edge and tilted views')
