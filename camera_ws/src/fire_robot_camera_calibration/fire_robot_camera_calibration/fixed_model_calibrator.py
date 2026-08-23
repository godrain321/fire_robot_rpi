"""Monocular calibration GUI with an explicitly selected camera model."""

import argparse
import functools
import re

from camera_calibration.calibrator import (
    CAMERA_MODEL,
    ChessboardInfo,
    MonoCalibrator,
    Patterns,
)
from camera_calibration.camera_calibrator import OpenCVCalibrationNode
import cv2
from message_filters import ApproximateTimeSynchronizer, TimeSynchronizer
import rclpy


_BOARD_SIZE_PATTERN = re.compile(r'^[1-9][0-9]*x[1-9][0-9]*$')


def camera_model_from_name(name):
    """Convert a launch-friendly model name to camera_calibration's enum."""
    normalized = name.strip().lower()
    if normalized == 'fisheye':
        return CAMERA_MODEL.FISHEYE
    if normalized == 'pinhole':
        return CAMERA_MODEL.PINHOLE
    raise ValueError(
        f'Unsupported camera model {name!r}; use "fisheye" or "pinhole".'
    )


def parse_board_size(value):
    """Parse an inner-corner count such as ``8x9``."""
    if not _BOARD_SIZE_PATTERN.fullmatch(value):
        raise argparse.ArgumentTypeError(
            'board size must be INNER_COLUMNSxINNER_ROWS, for example 8x9'
        )
    columns, rows = (int(part) for part in value.split('x'))
    if columns < 2 or rows < 2:
        raise argparse.ArgumentTypeError(
            'board size must contain at least 2x2 inner corners'
        )
    return columns, rows


class FixedModelCalibrationNode(OpenCVCalibrationNode):
    """Use one distortion model and omit the error-prone model trackbar."""

    def __init__(self, *args, camera_model, **kwargs):
        self._fixed_camera_model = camera_model
        self._model_was_reported = False
        super().__init__(*args, **kwargs)

    def initWindow(self):  # noqa: N802 - inherited ROS package API
        cv2.namedWindow('display', cv2.WINDOW_NORMAL)
        cv2.setMouseCallback('display', self.on_mouse)
        cv2.createTrackbar('scale', 'display', 0, 100, self.on_scale)

    def handle_monocular(self, msg):
        """Create the calibrator with the selected model before frame one."""
        if self.c is None:
            self.c = MonoCalibrator(
                self._boards,
                self._calib_flags,
                self._fisheye_calib_flags,
                self._pattern,
                name=self._camera_name,
                checkerboard_flags=self._checkerboard_flags,
                max_chessboard_speed=self._max_chessboard_speed,
            )
            self.c.set_cammodel(self._fixed_camera_model)

        if not self._model_was_reported:
            model_name = self._fixed_camera_model.name.lower()
            self.get_logger().info(
                f'Calibration distortion model locked to: {model_name}'
            )
            self._model_was_reported = True

        drawable = self.c.handle_msg(msg)
        self.displaywidth = drawable.scrib.shape[1]
        self.redraw_monocular(drawable)


def _argument_parser():
    parser = argparse.ArgumentParser(
        description='Monocular calibration with a fixed lens model.'
    )
    parser.add_argument('--size', default='8x9', type=parse_board_size)
    parser.add_argument('--square', default=0.07, type=float)
    parser.add_argument('--camera-name', default='camera')
    parser.add_argument(
        '--camera-model',
        choices=('fisheye', 'pinhole'),
        default='fisheye',
    )
    parser.add_argument('--max-chessboard-speed', default=-1.0, type=float)
    parser.add_argument('--queue-size', default=1, type=int)
    parser.add_argument('--approximate', default=0.0, type=float)
    return parser


def main(args=None):
    """Run the fixed-model monocular calibration GUI."""
    ros_args = rclpy.utilities.remove_ros_args(args=args)
    options = _argument_parser().parse_args(ros_args[1:])
    if options.square <= 0.0:
        raise ValueError('--square must be greater than zero')

    columns, rows = options.size
    boards = [
        ChessboardInfo(
            'chessboard',
            columns,
            rows,
            options.square,
        )
    ]
    camera_model = camera_model_from_name(options.camera_model)

    fisheye_flags = 0
    if camera_model == CAMERA_MODEL.FISHEYE:
        fisheye_flags = (
            cv2.fisheye.CALIB_RECOMPUTE_EXTRINSIC
            | cv2.fisheye.CALIB_FIX_SKEW
        )

    synchronizer = TimeSynchronizer
    if options.approximate > 0.0:
        synchronizer = functools.partial(
            ApproximateTimeSynchronizer,
            slop=options.approximate,
        )

    rclpy.init(args=args)
    node = FixedModelCalibrationNode(
        'fixed_model_cameracalibrator',
        boards,
        False,
        synchronizer,
        0,
        fisheye_flags,
        Patterns.Chessboard,
        options.camera_name,
        checkerboard_flags=cv2.CALIB_CB_FAST_CHECK,
        max_chessboard_speed=options.max_chessboard_speed,
        queue_size=options.queue_size,
        camera_model=camera_model,
    )
    try:
        node.get_logger().info(
            'Board inner corners: '
            f'{columns}x{rows}; square edge: {options.square:.6f} m'
        )
        node.spin()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
