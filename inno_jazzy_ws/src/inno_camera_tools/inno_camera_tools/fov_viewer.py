"""Live Camera Module 3 field-of-view viewer and snapshot recorder."""

from datetime import datetime
import math
from pathlib import Path
from typing import Optional, Tuple

import cv2
from cv_bridge import CvBridge
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
import yaml

from .fov_geometry import (
    field_of_view_deg,
    object_size_px,
    plane_coverage_m,
    scaled_focal_lengths,
)


class FovViewer(Node):
    """Show live video with distance-dependent coverage annotations."""

    def __init__(self) -> None:
        super().__init__('camera_fov_viewer')
        defaults = {
            'image_topic': '/camera/image_raw',
            'camera_info_topic': '/camera/camera_info',
            'calibration_file': '',
            'output_dir': '~/fire_robot_fov_check',
            'target_distance_m': 2.0,
            'distance_step_m': 0.25,
            'reference_person_height_m': 1.70,
            'reference_person_width_m': 0.50,
            'display_scale': 0.85,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

        self.image_topic = str(self.get_parameter('image_topic').value)
        self.camera_info_topic = str(
            self.get_parameter('camera_info_topic').value
        )
        self.output_dir = Path(
            str(self.get_parameter('output_dir').value)
        ).expanduser()
        self.distance_m = float(
            self.get_parameter('target_distance_m').value
        )
        self.distance_step_m = float(
            self.get_parameter('distance_step_m').value
        )
        self.person_height_m = float(
            self.get_parameter('reference_person_height_m').value
        )
        self.person_width_m = float(
            self.get_parameter('reference_person_width_m').value
        )
        self.display_scale = float(
            self.get_parameter('display_scale').value
        )
        numeric = (
            self.distance_m,
            self.distance_step_m,
            self.person_height_m,
            self.person_width_m,
            self.display_scale,
        )
        if not all(math.isfinite(value) and value > 0.0 for value in numeric):
            raise ValueError(
                'distance, reference size, and display scale must be positive'
            )

        self.output_dir.mkdir(parents=True, exist_ok=True)
        self.bridge = CvBridge()
        self.intrinsics: Optional[Tuple[float, float, int, int]] = None
        self.intrinsics_source = 'none'
        calibration_file = str(
            self.get_parameter('calibration_file').value
        ).strip()
        if calibration_file:
            self._load_calibration(Path(calibration_file).expanduser())

        self.create_subscription(
            CameraInfo,
            self.camera_info_topic,
            self._camera_info_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Image,
            self.image_topic,
            self._image_callback,
            qos_profile_sensor_data,
        )
        self.window_name = 'Fire robot camera FOV check'
        try:
            cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
        except cv2.error as error:
            raise RuntimeError(
                'OpenCV preview cannot open a window. Run from the Pi desktop '
                'or use SSH X forwarding.'
            ) from error
        self.get_logger().info(
            f'Waiting for {self.image_topic}; output={self.output_dir}'
        )
        self.get_logger().info(
            'Keys: s/SPACE=save raw+annotated, +/-=distance, q=quit'
        )

    def _load_calibration(self, path: Path) -> None:
        try:
            with path.open(encoding='utf-8') as stream:
                document = yaml.safe_load(stream)
            matrix = document['camera_matrix']['data']
            width = int(document['image_width'])
            height = int(document['image_height'])
            focal_x = float(matrix[0])
            focal_y = float(matrix[4])
            field_of_view_deg(width, height, focal_x, focal_y)
        except (
            OSError, KeyError, TypeError, ValueError, yaml.YAMLError
        ) as error:
            raise ValueError(
                f'cannot read camera calibration {path}: {error}'
            ) from error
        self.intrinsics = (focal_x, focal_y, width, height)
        self.intrinsics_source = path.name
        self.get_logger().info(f'Loaded camera intrinsics from {path}')

    def _camera_info_callback(self, message: CameraInfo) -> None:
        try:
            focal_x = float(message.k[0])
            focal_y = float(message.k[4])
            width = int(message.width)
            height = int(message.height)
            field_of_view_deg(width, height, focal_x, focal_y)
        except (IndexError, TypeError, ValueError):
            return
        self.intrinsics = (focal_x, focal_y, width, height)
        self.intrinsics_source = self.camera_info_topic

    def _geometry(
        self, frame_width: int, frame_height: int
    ) -> Optional[Tuple[float, float, float, float]]:
        if self.intrinsics is None:
            return None
        focal_x, focal_y, source_width, source_height = self.intrinsics
        focal_x, focal_y = scaled_focal_lengths(
            focal_x,
            focal_y,
            source_width,
            source_height,
            frame_width,
            frame_height,
        )
        horizontal, vertical = field_of_view_deg(
            frame_width, frame_height, focal_x, focal_y
        )
        return focal_x, focal_y, horizontal, vertical

    @staticmethod
    def _text(frame, text: str, row: int, color=(0, 255, 255)) -> None:
        cv2.putText(
            frame,
            text,
            (18, 30 + row * 27),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.65,
            color,
            2,
            cv2.LINE_AA,
        )

    def _annotate(self, frame):
        annotated = frame.copy()
        height, width = annotated.shape[:2]
        grid_color = (80, 220, 80)
        for column in (1, 2):
            x = round(width * column / 3.0)
            cv2.line(annotated, (x, 0), (x, height - 1), grid_color, 1)
        for row in (1, 2):
            y = round(height * row / 3.0)
            cv2.line(annotated, (0, y), (width - 1, y), grid_color, 1)
        center = (width // 2, height // 2)
        cv2.drawMarker(
            annotated, center, (0, 255, 255), cv2.MARKER_CROSS, 28, 2
        )

        self._text(
            annotated,
            f'{width}x{height} | distance={self.distance_m:.2f} m',
            0,
        )
        geometry = self._geometry(width, height)
        if geometry is None:
            self._text(
                annotated, 'NO VALID CAMERA INTRINSICS', 1, (0, 0, 255)
            )
        else:
            focal_x, focal_y, horizontal, vertical = geometry
            scene_width, scene_height = plane_coverage_m(
                self.distance_m, horizontal, vertical
            )
            person_width_px, person_height_px = object_size_px(
                self.distance_m,
                self.person_width_m,
                self.person_height_m,
                focal_x,
                focal_y,
            )
            self._text(
                annotated,
                f'FOV H={horizontal:.1f} deg V={vertical:.1f} deg '
                f'({self.intrinsics_source})',
                1,
            )
            self._text(
                annotated,
                f'plane coverage ~= {scene_width:.2f} m x '
                f'{scene_height:.2f} m',
                2,
            )
            self._text(
                annotated,
                f'{self.person_height_m:.2f} m person ~= '
                f'{person_height_px:.0f}px '
                f'({person_height_px / height * 100.0:.0f}% height)',
                3,
            )
            box_width = max(1, round(person_width_px))
            box_height = max(1, round(person_height_px))
            left = max(0, center[0] - box_width // 2)
            right = min(width - 1, center[0] + box_width // 2)
            top = max(0, center[1] - box_height // 2)
            bottom = min(height - 1, center[1] + box_height // 2)
            cv2.rectangle(
                annotated, (left, top), (right, bottom), (255, 180, 0), 2
            )
        self._text(
            annotated,
            's/SPACE save | +/- distance | q quit',
            4,
            (255, 255, 255),
        )
        return annotated

    def _save(self, raw, annotated) -> None:
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S_%f')[:-3]
        distance = f'{self.distance_m:.2f}m'.replace('.', 'p')
        stem = f'fov_{timestamp}_d{distance}'
        raw_path = self.output_dir / f'{stem}_raw.png'
        annotated_path = self.output_dir / f'{stem}_annotated.png'
        raw_ok = cv2.imwrite(str(raw_path), raw)
        annotated_ok = cv2.imwrite(str(annotated_path), annotated)
        if raw_ok and annotated_ok:
            self.get_logger().info(
                f'Saved {raw_path.name} and {annotated_path.name}'
            )
        else:
            self.get_logger().error(
                f'Failed to save snapshot under {self.output_dir}'
            )

    def _image_callback(self, message: Image) -> None:
        try:
            frame = self.bridge.imgmsg_to_cv2(
                message, desired_encoding='bgr8'
            )
        except Exception as error:  # cv_bridge has backend-specific exceptions
            self.get_logger().error(f'Image decode failed: {error}')
            return
        annotated = self._annotate(frame)
        if self.display_scale != 1.0:
            annotated_for_display = cv2.resize(
                annotated,
                None,
                fx=self.display_scale,
                fy=self.display_scale,
                interpolation=cv2.INTER_AREA,
            )
        else:
            annotated_for_display = annotated
        cv2.imshow(self.window_name, annotated_for_display)
        key = cv2.waitKey(1) & 0xFF
        if key in (ord('s'), ord(' ')):
            self._save(frame, annotated)
        elif key in (ord('+'), ord('=')):
            self.distance_m += self.distance_step_m
            self.get_logger().info(
                f'Distance set to {self.distance_m:.2f} m'
            )
        elif key in (ord('-'), ord('_')):
            self.distance_m = max(
                self.distance_step_m,
                self.distance_m - self.distance_step_m,
            )
            self.get_logger().info(
                f'Distance set to {self.distance_m:.2f} m'
            )
        elif key == ord('q') or cv2.getWindowProperty(
            self.window_name, cv2.WND_PROP_VISIBLE
        ) < 1:
            rclpy.shutdown()

    def destroy_node(self) -> None:
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = FovViewer()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except (RuntimeError, ValueError) as error:
        print(f'camera_fov_viewer: {error}', flush=True)
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
