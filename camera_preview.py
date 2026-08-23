#!/usr/bin/env python3
"""Open Camera Module 3, preview raw frames, and save on the S key."""

import argparse
from datetime import datetime
from pathlib import Path
import sys


def positive_integer(text):
    value = int(text)
    if value <= 0:
        raise argparse.ArgumentTypeError('must be greater than zero')
    return value


def parse_args(argv=None):
    parser = argparse.ArgumentParser(
        description='Simple Raspberry Pi Camera Module 3 preview and capture.'
    )
    parser.add_argument('--camera', type=int, default=0, help='camera index')
    parser.add_argument('--width', type=positive_integer, default=1280)
    parser.add_argument('--height', type=positive_integer, default=720)
    parser.add_argument(
        '--output-dir',
        type=Path,
        default=Path(__file__).resolve().parent / 'data/camera_capture',
    )
    return parser.parse_args(argv)


def capture_filename(output_dir, sequence, now=None):
    timestamp = (now or datetime.now()).strftime('%Y%m%d_%H%M%S_%f')[:-3]
    return output_dir / f'camera_{timestamp}_{sequence:03d}.jpg'


def load_camera_dependencies():
    try:
        import cv2
        from libcamera import controls
        from picamera2 import Picamera2
    except ImportError as error:
        raise RuntimeError(
            'Picamera2/OpenCV is missing. Install it with: '
            'sudo apt install python3-picamera2 python3-opencv'
        ) from error
    return cv2, controls, Picamera2


def select_camera(Picamera2, index):
    cameras = Picamera2.global_camera_info()
    if not cameras:
        raise RuntimeError(
            'No Raspberry Pi CSI camera found. Power off the Pi and check '
            'the Camera Module 3 cable.'
        )
    if index < 0 or index >= len(cameras):
        raise RuntimeError(
            f'camera index {index} does not exist; '
            f'detected {len(cameras)} camera(s)'
        )
    model = str(cameras[index].get('Model', 'unknown'))
    if 'imx708' not in model.casefold():
        raise RuntimeError(
            f'camera {index} is {model}, not Camera Module 3 (IMX708)'
        )
    return model


def run(args):
    cv2, controls, Picamera2 = load_camera_dependencies()
    model = select_camera(Picamera2, args.camera)
    output_dir = args.output_dir.expanduser().resolve()
    output_dir.mkdir(parents=True, exist_ok=True)

    camera = Picamera2(args.camera)
    configuration = camera.create_preview_configuration(
        main={
            'size': (args.width, args.height),
            'format': 'RGB888',
        },
        buffer_count=4,
    )
    camera.configure(configuration)
    try:
        camera.set_controls({'AfMode': controls.AfModeEnum.Continuous})
    except (AttributeError, RuntimeError):
        print(
            'Continuous autofocus unavailable; using camera default.',
            flush=True,
        )

    window_name = f'Camera Module 3 - {model}'
    sequence = 0
    camera.start()
    print(f'Camera opened: {model} ({args.width}x{args.height})', flush=True)
    print(f'Photos: {output_dir}', flush=True)
    print('Keys: s=save photo, q/ESC=quit', flush=True)
    try:
        cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
        while True:
            frame = camera.capture_array('main')
            cv2.imshow(window_name, frame)
            key = cv2.waitKey(1) & 0xFF
            if key in (ord('s'), ord('S')):
                sequence += 1
                path = capture_filename(output_dir, sequence)
                if not cv2.imwrite(
                    str(path), frame, [cv2.IMWRITE_JPEG_QUALITY, 95]
                ):
                    raise RuntimeError(f'failed to save photo: {path}')
                print(f'Saved #{sequence}: {path}', flush=True)
            elif key in (ord('q'), ord('Q'), 27):
                break
            if cv2.getWindowProperty(window_name, cv2.WND_PROP_VISIBLE) < 1:
                break
    finally:
        camera.stop()
        camera.close()
        cv2.destroyAllWindows()


def main(argv=None):
    args = parse_args(argv)
    try:
        run(args)
    except KeyboardInterrupt:
        pass
    except RuntimeError as error:
        print(f'camera_preview: {error}', file=sys.stderr)
        return 1
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
