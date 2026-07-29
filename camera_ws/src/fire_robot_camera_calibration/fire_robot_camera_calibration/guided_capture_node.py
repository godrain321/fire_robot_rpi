"""Guided checkerboard image capture for offline fisheye calibration."""

import csv
import math
from pathlib import Path
import time

import cv2
from cv_bridge import CvBridge
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage, Image


class GuidedCaptureNode(Node):
    """Capture sharp, spatially diverse checkerboard images."""

    def __init__(self):
        super().__init__('guided_checkerboard_capture')

        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('transport', 'raw')
        self.declare_parameter('output_dir', 'calibration_images')
        self.declare_parameter('board_cols', 8)
        self.declare_parameter('board_rows', 9)
        self.declare_parameter('max_images', 80)
        self.declare_parameter('minimum_interval_s', 0.6)
        self.declare_parameter('preview', True)
        self.declare_parameter('auto_save', True)
        self.declare_parameter('blur_threshold', 35.0)
        self.declare_parameter('display_scale', 0.65)

        self.image_topic = str(self.get_parameter('image_topic').value)
        self.transport = str(self.get_parameter('transport').value).lower()
        self.output_dir = Path(
            str(self.get_parameter('output_dir').value)
        ).expanduser()
        self.board_cols = int(self.get_parameter('board_cols').value)
        self.board_rows = int(self.get_parameter('board_rows').value)
        self.max_images = int(self.get_parameter('max_images').value)
        self.minimum_interval = float(
            self.get_parameter('minimum_interval_s').value
        )
        self.preview = bool(self.get_parameter('preview').value)
        self.auto_save = bool(self.get_parameter('auto_save').value)
        self.blur_threshold = float(
            self.get_parameter('blur_threshold').value
        )
        self.display_scale = float(
            self.get_parameter('display_scale').value
        )

        if self.transport not in ('raw', 'compressed'):
            raise ValueError("transport must be 'raw' or 'compressed'")
        if self.board_cols < 2 or self.board_rows < 2:
            raise ValueError('Checkerboard dimensions must be at least 2x2')
        if self.max_images < 1:
            raise ValueError('max_images must be positive')

        self.output_dir.mkdir(parents=True, exist_ok=True)
        self.pattern_size = (self.board_cols, self.board_rows)
        self.bridge = CvBridge()
        self.count = len(list(self.output_dir.glob('calib_*.png')))
        self.last_save_time = 0.0
        self.saved_pose_vectors = []
        self.force_save = False
        self.zone_counts = {
            f'{row},{column}': 0
            for row in range(3)
            for column in range(3)
        }

        self.csv_stream = (
            self.output_dir / 'capture_stats.csv'
        ).open('a', newline='', encoding='utf-8')
        self.csv_writer = csv.writer(self.csv_stream)
        if self.csv_stream.tell() == 0:
            self.csv_writer.writerow(
                [
                    'filename',
                    'unix_time',
                    'center_x_norm',
                    'center_y_norm',
                    'zone',
                    'area_ratio',
                    'tilt_score',
                    'roll_deg',
                    'blur_score',
                    'save_reason',
                ]
            )

        message_type = Image if self.transport == 'raw' else CompressedImage
        self.subscription = self.create_subscription(
            message_type,
            self.image_topic,
            self._image_callback,
            qos_profile_sensor_data,
        )

        self.get_logger().info(
            f'Listening to {self.image_topic} ({self.transport})'
        )
        self.get_logger().info(f'Saving images under {self.output_dir}')
        self.get_logger().info(
            f'Checkerboard inner corners: {self.board_cols}x{self.board_rows}'
        )
        if self.preview:
            self.get_logger().info('Keys: s=force save, q=quit')

    def _decode(self, message):
        if self.transport == 'raw':
            return self.bridge.imgmsg_to_cv2(
                message,
                desired_encoding='bgr8',
            )
        encoded = np.frombuffer(message.data, dtype=np.uint8)
        return cv2.imdecode(encoded, cv2.IMREAD_COLOR)

    def _detect_corners(self, gray):
        flags = (
            cv2.CALIB_CB_ADAPTIVE_THRESH
            | cv2.CALIB_CB_NORMALIZE_IMAGE
            | cv2.CALIB_CB_FAST_CHECK
        )
        found, corners = cv2.findChessboardCorners(
            gray,
            self.pattern_size,
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
            gray,
            corners,
            winSize=(11, 11),
            zeroZone=(-1, -1),
            criteria=criteria,
        )

    def _statistics(self, gray, corners):
        height, width = gray.shape[:2]
        points = corners.reshape(-1, 2)
        center = points.mean(axis=0)
        center_x = float(center[0] / width)
        center_y = float(center[1] / height)
        column = min(2, max(0, int(center_x * 3)))
        row = min(2, max(0, int(center_y * 3)))
        zone = f'{row},{column}'

        hull = cv2.convexHull(points.astype(np.float32))
        area_ratio = float(
            cv2.contourArea(hull) / float(width * height)
        )

        top_left = points[0]
        top_right = points[self.board_cols - 1]
        bottom_left = points[(self.board_rows - 1) * self.board_cols]
        bottom_right = points[-1]
        epsilon = 1e-9
        top_length = np.linalg.norm(top_right - top_left) + epsilon
        bottom_length = np.linalg.norm(
            bottom_right - bottom_left
        ) + epsilon
        left_length = np.linalg.norm(bottom_left - top_left) + epsilon
        right_length = np.linalg.norm(
            bottom_right - top_right
        ) + epsilon
        tilt = max(
            abs(math.log(top_length / bottom_length)),
            abs(math.log(left_length / right_length)),
        )
        roll = math.degrees(
            math.atan2(
                top_right[1] - top_left[1],
                top_right[0] - top_left[0],
            )
        )
        blur = float(cv2.Laplacian(gray, cv2.CV_64F).var())

        return {
            'center_x': center_x,
            'center_y': center_y,
            'zone': zone,
            'area_ratio': area_ratio,
            'tilt': tilt,
            'roll': roll,
            'blur': blur,
        }

    @staticmethod
    def _pose_vector(statistics):
        return np.array(
            [
                statistics['center_x'],
                statistics['center_y'],
                statistics['area_ratio'] * 2.0,
                statistics['tilt'],
                statistics['roll'] / 90.0,
            ],
            dtype=np.float64,
        )

    def _is_diverse(self, statistics):
        candidate = self._pose_vector(statistics)
        if not self.saved_pose_vectors:
            return True
        distances = [
            np.linalg.norm(candidate - saved)
            for saved in self.saved_pose_vectors[-25:]
        ]
        return min(distances) >= 0.065

    def _save_image(self, frame, statistics, reason):
        filename = f'calib_{self.count:03d}.png'
        path = self.output_dir / filename
        if not cv2.imwrite(str(path), frame):
            self.get_logger().error(f'Failed to write {path}')
            return

        self.csv_writer.writerow(
            [
                filename,
                f'{time.time():.3f}',
                f'{statistics["center_x"]:.5f}',
                f'{statistics["center_y"]:.5f}',
                statistics['zone'],
                f'{statistics["area_ratio"]:.6f}',
                f'{statistics["tilt"]:.5f}',
                f'{statistics["roll"]:.3f}',
                f'{statistics["blur"]:.2f}',
                reason,
            ]
        )
        self.csv_stream.flush()
        self.zone_counts[statistics['zone']] += 1
        self.saved_pose_vectors.append(self._pose_vector(statistics))
        self.count += 1
        self.last_save_time = time.monotonic()
        self.get_logger().info(
            f'Saved {filename} ({self.count}/{self.max_images}): {reason}'
        )

    def _draw_status(self, frame, corners, statistics):
        display = frame.copy()
        if corners is not None:
            cv2.drawChessboardCorners(
                display,
                self.pattern_size,
                corners,
                True,
            )
        status = (
            f'{self.count}/{self.max_images} | '
            f'zone={statistics["zone"]} '
            f'blur={statistics["blur"]:.0f}'
            if statistics
            else f'{self.count}/{self.max_images} | checkerboard not found'
        )
        cv2.putText(
            display,
            status,
            (20, 35),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.75,
            (0, 255, 0) if statistics else (0, 0, 255),
            2,
        )
        cv2.putText(
            display,
            'Cover all 3x3 zones; vary distance, tilt and roll',
            (20, 68),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (0, 255, 255),
            2,
        )
        cv2.putText(
            display,
            's: force save   q: quit',
            (20, 98),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (0, 255, 255),
            2,
        )
        return cv2.resize(
            display,
            None,
            fx=self.display_scale,
            fy=self.display_scale,
        )

    def _image_callback(self, message):
        frame = self._decode(message)
        if frame is None:
            self.get_logger().warning('Image decode failed')
            return

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners = self._detect_corners(gray)
        statistics = (
            self._statistics(gray, corners)
            if corners is not None
            else None
        )

        if statistics is not None and self.count < self.max_images:
            elapsed = time.monotonic() - self.last_save_time
            sharp = statistics['blur'] >= self.blur_threshold
            diverse = self._is_diverse(statistics)
            zone_needed = self.zone_counts[statistics['zone']] < 10
            can_auto_save = (
                self.auto_save
                and elapsed >= self.minimum_interval
                and sharp
                and diverse
                and zone_needed
            )
            if can_auto_save or self.force_save:
                reason = 'forced' if self.force_save else 'diverse pose'
                self._save_image(frame, statistics, reason)
                self.force_save = False

        if self.preview:
            cv2.imshow(
                'fire robot intrinsic capture',
                self._draw_status(frame, corners, statistics),
            )
            key = cv2.waitKey(1) & 0xFF
            if key == ord('s'):
                self.force_save = True
            elif key == ord('q'):
                rclpy.shutdown()
                return

        if self.count >= self.max_images:
            self.get_logger().info('Target image count reached')
            rclpy.shutdown()

    def destroy_node(self):
        self.csv_stream.close()
        super().destroy_node()


def main():
    """Run the guided capture node."""
    rclpy.init()
    node = GuidedCaptureNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
