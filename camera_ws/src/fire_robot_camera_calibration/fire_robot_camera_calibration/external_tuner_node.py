"""Interactive 2D LiDAR-to-camera extrinsic calibration overlay."""

import math
from pathlib import Path

import cv2
from cv_bridge import CvBridge
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, LaserScan
import yaml

from .calibration_math import camera_lidar_transform


class ExternalTunerNode(Node):
    """Tune a rigid transform by projecting a planar laser scan on an image."""

    def __init__(self):
        super().__init__('lidar_camera_external_tuner')

        self.declare_parameter('image_topic', '/camera/image_rect')
        self.declare_parameter(
            'camera_info_topic',
            '/camera/camera_info_rect',
        )
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('initial_yaml', '')
        self.declare_parameter('output_yaml', 'extrinsic_result.yaml')
        self.declare_parameter('camera_frame', 'camera_optical_frame')
        self.declare_parameter('lidar_frame', 'laser_frame')
        self.declare_parameter('min_range_m', 0.3)
        self.declare_parameter('max_range_m', 4.0)
        self.declare_parameter('angle_min_deg', -70.0)
        self.declare_parameter('angle_max_deg', 70.0)
        self.declare_parameter('camera_x_m', 0.0)
        self.declare_parameter('camera_y_m', 0.0)
        self.declare_parameter('camera_z_m', 0.15)
        self.declare_parameter('roll_deg', 0.0)
        self.declare_parameter('pitch_deg', 0.0)
        self.declare_parameter('yaw_deg', 0.0)
        self.declare_parameter('translation_step_m', 0.01)
        self.declare_parameter('rotation_step_deg', 1.0)
        self.declare_parameter('display_scale', 0.7)

        self.image_topic = str(self.get_parameter('image_topic').value)
        self.camera_info_topic = str(
            self.get_parameter('camera_info_topic').value
        )
        self.scan_topic = str(self.get_parameter('scan_topic').value)
        self.initial_yaml = Path(
            str(self.get_parameter('initial_yaml').value)
        ).expanduser()
        self.output_yaml = Path(
            str(self.get_parameter('output_yaml').value)
        ).expanduser()
        self.camera_frame = str(
            self.get_parameter('camera_frame').value
        )
        self.lidar_frame = str(self.get_parameter('lidar_frame').value)
        self.min_range = float(self.get_parameter('min_range_m').value)
        self.max_range = float(self.get_parameter('max_range_m').value)
        self.angle_min = float(
            self.get_parameter('angle_min_deg').value
        )
        self.angle_max = float(
            self.get_parameter('angle_max_deg').value
        )
        self.camera_position = np.array(
            [
                float(self.get_parameter('camera_x_m').value),
                float(self.get_parameter('camera_y_m').value),
                float(self.get_parameter('camera_z_m').value),
            ],
            dtype=np.float64,
        )
        self.roll = float(self.get_parameter('roll_deg').value)
        self.pitch = float(self.get_parameter('pitch_deg').value)
        self.yaw = float(self.get_parameter('yaw_deg').value)
        self.translation_step = float(
            self.get_parameter('translation_step_m').value
        )
        self.rotation_step = float(
            self.get_parameter('rotation_step_deg').value
        )
        self.display_scale = float(
            self.get_parameter('display_scale').value
        )

        if self.initial_yaml.is_file():
            self._load_initial_guess(self.initial_yaml)
        if self.min_range < 0.0 or self.max_range <= self.min_range:
            raise ValueError('Invalid scan range filter')
        if self.angle_max <= self.angle_min:
            raise ValueError('Invalid scan angle filter')

        self.bridge = CvBridge()
        self.latest_image = None
        self.latest_scan = None
        self.camera_matrix = None

        self.image_subscription = self.create_subscription(
            Image,
            self.image_topic,
            self._image_callback,
            qos_profile_sensor_data,
        )
        self.info_subscription = self.create_subscription(
            CameraInfo,
            self.camera_info_topic,
            self._camera_info_callback,
            qos_profile_sensor_data,
        )
        self.scan_subscription = self.create_subscription(
            LaserScan,
            self.scan_topic,
            self._scan_callback,
            qos_profile_sensor_data,
        )
        self.timer = self.create_timer(0.03, self._draw)

        self.get_logger().info(f'Image: {self.image_topic}')
        self.get_logger().info(f'CameraInfo: {self.camera_info_topic}')
        self.get_logger().info(f'LaserScan: {self.scan_topic}')
        self.get_logger().info(
            'Keys: q quit | p save | w/s x | a/d y | r/f z | '
            'i/k pitch | j/l yaw | u/o roll'
        )

    def _load_initial_guess(self, path):
        with path.open('r', encoding='utf-8') as stream:
            data = yaml.safe_load(stream)

        position = data.get('camera_position_in_laser_frame_m', {})
        correction = data.get(
            'correction_rotation_deg_in_camera_optical',
            {},
        )
        scan_filter = data.get('scan_filter', {})
        self.camera_position = np.array(
            [
                position.get('x_forward', self.camera_position[0]),
                position.get('y_left', self.camera_position[1]),
                position.get('z_up', self.camera_position[2]),
            ],
            dtype=np.float64,
        )
        self.roll = float(correction.get('roll', self.roll))
        self.pitch = float(correction.get('pitch', self.pitch))
        self.yaw = float(correction.get('yaw', self.yaw))
        self.min_range = float(
            scan_filter.get('min_range_m', self.min_range)
        )
        self.max_range = float(
            scan_filter.get('max_range_m', self.max_range)
        )
        self.angle_min = float(
            scan_filter.get('angle_min_deg', self.angle_min)
        )
        self.angle_max = float(
            scan_filter.get('angle_max_deg', self.angle_max)
        )
        self.get_logger().info(f'Loaded initial guess: {path}')

    def _image_callback(self, message):
        self.latest_image = self.bridge.imgmsg_to_cv2(
            message,
            desired_encoding='bgr8',
        )

    def _camera_info_callback(self, message):
        self.camera_matrix = np.asarray(
            message.k,
            dtype=np.float64,
        ).reshape(3, 3)

    def _scan_callback(self, message):
        self.latest_scan = message

    def _scan_points(self):
        points = []
        for index, distance in enumerate(self.latest_scan.ranges):
            if not math.isfinite(distance):
                continue
            if distance < self.min_range or distance > self.max_range:
                continue

            angle = (
                self.latest_scan.angle_min
                + index * self.latest_scan.angle_increment
            )
            angle_degrees = math.degrees(angle)
            if angle_degrees < self.angle_min:
                continue
            if angle_degrees > self.angle_max:
                continue
            points.append(
                [
                    distance * math.cos(angle),
                    distance * math.sin(angle),
                    0.0,
                    distance,
                ]
            )
        if not points:
            return np.empty((0, 4), dtype=np.float64)
        return np.asarray(points, dtype=np.float64)

    def _project(self, laser_points):
        rotation, translation = camera_lidar_transform(
            self.camera_position,
            self.roll,
            self.pitch,
            self.yaw,
        )
        camera_points = (
            rotation @ laser_points[:, :3].T
        ).T + translation

        valid = camera_points[:, 2] > 0.05
        camera_points = camera_points[valid]
        distances = laser_points[valid, 3]
        if camera_points.size == 0:
            return []

        projected_x = (
            self.camera_matrix[0, 0]
            * camera_points[:, 0]
            / camera_points[:, 2]
            + self.camera_matrix[0, 2]
        )
        projected_y = (
            self.camera_matrix[1, 1]
            * camera_points[:, 1]
            / camera_points[:, 2]
            + self.camera_matrix[1, 2]
        )
        return list(zip(projected_x, projected_y, distances))

    def _save(self):
        rotation, translation = camera_lidar_transform(
            self.camera_position,
            self.roll,
            self.pitch,
            self.yaw,
        )
        data = {
            'description': (
                f'Manual extrinsic transform from {self.lidar_frame} '
                f'to {self.camera_frame}'
            ),
            'lidar_frame': self.lidar_frame,
            'camera_frame': self.camera_frame,
            'camera_position_in_laser_frame_m': {
                'x_forward': float(self.camera_position[0]),
                'y_left': float(self.camera_position[1]),
                'z_up': float(self.camera_position[2]),
            },
            'correction_rotation_deg_in_camera_optical': {
                'roll': float(self.roll),
                'pitch': float(self.pitch),
                'yaw': float(self.yaw),
            },
            'T_camera_lidar': {
                'R_row_major': rotation.reshape(-1).tolist(),
                't_xyz': translation.tolist(),
            },
            'scan_filter': {
                'min_range_m': self.min_range,
                'max_range_m': self.max_range,
                'angle_min_deg': self.angle_min,
                'angle_max_deg': self.angle_max,
            },
        }
        self.output_yaml.parent.mkdir(parents=True, exist_ok=True)
        with self.output_yaml.open('w', encoding='utf-8') as stream:
            yaml.safe_dump(data, stream, sort_keys=False)
        self.get_logger().info(f'Saved extrinsic: {self.output_yaml}')

    @staticmethod
    def _put(image, text, y, color=(0, 255, 255)):
        cv2.putText(
            image,
            text,
            (20, y),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.58,
            color,
            2,
        )

    def _handle_key(self, key):
        if key == ord('q'):
            rclpy.shutdown()
        elif key == ord('p'):
            self._save()
        elif key == ord('w'):
            self.camera_position[0] += self.translation_step
        elif key == ord('s'):
            self.camera_position[0] -= self.translation_step
        elif key == ord('a'):
            self.camera_position[1] += self.translation_step
        elif key == ord('d'):
            self.camera_position[1] -= self.translation_step
        elif key == ord('r'):
            self.camera_position[2] += self.translation_step
        elif key == ord('f'):
            self.camera_position[2] -= self.translation_step
        elif key == ord('i'):
            self.pitch += self.rotation_step
        elif key == ord('k'):
            self.pitch -= self.rotation_step
        elif key == ord('j'):
            self.yaw += self.rotation_step
        elif key == ord('l'):
            self.yaw -= self.rotation_step
        elif key == ord('u'):
            self.roll += self.rotation_step
        elif key == ord('o'):
            self.roll -= self.rotation_step

    def _draw(self):
        if self.latest_image is None:
            return

        display = self.latest_image.copy()
        if self.camera_matrix is None:
            self._put(
                display,
                'Waiting for rectified CameraInfo...',
                35,
                (0, 0, 255),
            )
        elif self.latest_scan is None:
            self._put(
                display,
                'Waiting for LaserScan...',
                35,
                (0, 0, 255),
            )
        else:
            points = self._scan_points()
            projections = self._project(points)
            height, width = display.shape[:2]
            visible = 0
            for x_value, y_value, distance in projections:
                x_pixel = int(round(x_value))
                y_pixel = int(round(y_value))
                if not (
                    0 <= x_pixel < width
                    and 0 <= y_pixel < height
                ):
                    continue
                if distance < 1.0:
                    color = (0, 0, 255)
                elif distance < 2.0:
                    color = (0, 165, 255)
                else:
                    color = (0, 255, 0)
                cv2.circle(
                    display,
                    (x_pixel, y_pixel),
                    4,
                    color,
                    -1,
                )
                visible += 1

            self._put(
                display,
                f'Projected: {visible}/{len(points)}',
                35,
            )
            self._put(
                display,
                'camera in laser [m] '
                f'x={self.camera_position[0]:.3f} '
                f'y={self.camera_position[1]:.3f} '
                f'z={self.camera_position[2]:.3f}',
                65,
            )
            self._put(
                display,
                'correction [deg] '
                f'roll={self.roll:.1f} '
                f'pitch={self.pitch:.1f} '
                f'yaw={self.yaw:.1f}',
                95,
            )
            self._put(
                display,
                'p save | w/s x | a/d y | r/f z | '
                'i/k pitch | j/l yaw | u/o roll | q quit',
                125,
            )

        shown = cv2.resize(
            display,
            None,
            fx=self.display_scale,
            fy=self.display_scale,
        )
        cv2.imshow('fire robot lidar-camera extrinsic tuner', shown)
        self._handle_key(cv2.waitKey(1) & 0xFF)


def main():
    """Run the interactive external calibration tuner."""
    rclpy.init()
    node = ExternalTunerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
