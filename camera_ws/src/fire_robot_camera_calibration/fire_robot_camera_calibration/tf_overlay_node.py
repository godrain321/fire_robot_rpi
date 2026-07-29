"""Validate a LiDAR-to-camera TF by drawing laser points on images."""

import math
from pathlib import Path
import time

import cv2
from cv_bridge import CvBridge
import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, LaserScan
from tf2_ros import Buffer, TransformListener

from .calibration_math import rotation_from_quaternion


class TfOverlayNode(Node):
    """Project LaserScan points through the current TF transform."""

    def __init__(self):
        super().__init__('lidar_camera_tf_overlay')

        self.declare_parameter('image_topic', '/camera/image_rect')
        self.declare_parameter(
            'camera_info_topic',
            '/camera/camera_info_rect',
        )
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('camera_frame', 'camera_optical_frame')
        self.declare_parameter('lidar_frame', 'laser_frame')
        self.declare_parameter('min_range_m', 0.3)
        self.declare_parameter('max_range_m', 4.0)
        self.declare_parameter('angle_min_deg', -70.0)
        self.declare_parameter('angle_max_deg', 70.0)
        self.declare_parameter('display_scale', 0.7)
        self.declare_parameter('screenshot_dir', 'calibration_results')

        self.image_topic = str(self.get_parameter('image_topic').value)
        self.info_topic = str(
            self.get_parameter('camera_info_topic').value
        )
        self.scan_topic = str(self.get_parameter('scan_topic').value)
        self.camera_frame = str(
            self.get_parameter('camera_frame').value
        )
        self.lidar_frame = str(
            self.get_parameter('lidar_frame').value
        )
        self.min_range = float(self.get_parameter('min_range_m').value)
        self.max_range = float(self.get_parameter('max_range_m').value)
        self.angle_min = float(
            self.get_parameter('angle_min_deg').value
        )
        self.angle_max = float(
            self.get_parameter('angle_max_deg').value
        )
        self.display_scale = float(
            self.get_parameter('display_scale').value
        )
        self.screenshot_dir = Path(
            str(self.get_parameter('screenshot_dir').value)
        ).expanduser()

        self.bridge = CvBridge()
        self.latest_image = None
        self.latest_scan = None
        self.camera_matrix = None
        self.transform_buffer = Buffer()
        self.transform_listener = TransformListener(
            self.transform_buffer,
            self,
        )

        self.image_subscription = self.create_subscription(
            Image,
            self.image_topic,
            self._image_callback,
            qos_profile_sensor_data,
        )
        self.info_subscription = self.create_subscription(
            CameraInfo,
            self.info_topic,
            self._info_callback,
            qos_profile_sensor_data,
        )
        self.scan_subscription = self.create_subscription(
            LaserScan,
            self.scan_topic,
            self._scan_callback,
            qos_profile_sensor_data,
        )
        self.timer = self.create_timer(0.03, self._draw)
        self.get_logger().info('Keys: c=capture screenshot, q=quit')

    def _image_callback(self, message):
        self.latest_image = self.bridge.imgmsg_to_cv2(
            message,
            desired_encoding='bgr8',
        )

    def _info_callback(self, message):
        self.camera_matrix = np.asarray(
            message.k,
            dtype=np.float64,
        ).reshape(3, 3)

    def _scan_callback(self, message):
        self.latest_scan = message

    def _laser_points(self):
        points = []
        for index, distance in enumerate(self.latest_scan.ranges):
            if not math.isfinite(distance):
                continue
            if not self.min_range <= distance <= self.max_range:
                continue
            angle = (
                self.latest_scan.angle_min
                + index * self.latest_scan.angle_increment
            )
            angle_degrees = math.degrees(angle)
            if not self.angle_min <= angle_degrees <= self.angle_max:
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

    def _transform(self):
        message = self.transform_buffer.lookup_transform(
            self.camera_frame,
            self.lidar_frame,
            rclpy.time.Time(),
            timeout=Duration(seconds=0.15),
        )
        translation_message = message.transform.translation
        rotation_message = message.transform.rotation
        rotation = rotation_from_quaternion(
            rotation_message.x,
            rotation_message.y,
            rotation_message.z,
            rotation_message.w,
        )
        translation = np.array(
            [
                translation_message.x,
                translation_message.y,
                translation_message.z,
            ],
            dtype=np.float64,
        )
        return rotation, translation

    def _project(self, points, rotation, translation):
        camera_points = (
            rotation @ points[:, :3].T
        ).T + translation
        valid = camera_points[:, 2] > 0.05
        camera_points = camera_points[valid]
        distances = points[valid, 3]
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

    @staticmethod
    def _put(image, text, y, color=(0, 255, 255)):
        cv2.putText(
            image,
            text,
            (20, y),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            color,
            2,
        )

    def _draw(self):
        if self.latest_image is None:
            return
        display = self.latest_image.copy()

        if self.camera_matrix is None:
            self._put(display, 'Waiting for CameraInfo...', 35, (0, 0, 255))
        elif self.latest_scan is None:
            self._put(display, 'Waiting for LaserScan...', 35, (0, 0, 255))
        else:
            try:
                rotation, translation = self._transform()
            except Exception as error:
                self._put(display, 'Waiting for TF...', 35, (0, 0, 255))
                self._put(display, str(error)[:100], 65, (0, 0, 255))
            else:
                points = self._laser_points()
                projections = self._project(
                    points,
                    rotation,
                    translation,
                )
                height, width = display.shape[:2]
                visible = 0
                for x_value, y_value, distance in projections:
                    pixel = (
                        int(round(x_value)),
                        int(round(y_value)),
                    )
                    if not (
                        0 <= pixel[0] < width
                        and 0 <= pixel[1] < height
                    ):
                        continue
                    if distance < 1.0:
                        color = (0, 0, 255)
                    elif distance < 2.0:
                        color = (0, 165, 255)
                    else:
                        color = (0, 255, 0)
                    cv2.circle(display, pixel, 4, color, -1)
                    visible += 1
                self._put(
                    display,
                    f'TF overlay: {visible}/{len(points)} points',
                    35,
                )
                self._put(
                    display,
                    f'{self.camera_frame} <- {self.lidar_frame}',
                    65,
                )
                self._put(
                    display,
                    'red <1 m | orange 1-2 m | green >=2 m',
                    95,
                )

        shown = cv2.resize(
            display,
            None,
            fx=self.display_scale,
            fy=self.display_scale,
        )
        cv2.imshow('fire robot extrinsic validation', shown)
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            rclpy.shutdown()
        elif key == ord('c'):
            self.screenshot_dir.mkdir(parents=True, exist_ok=True)
            output = self.screenshot_dir / (
                f'extrinsic_overlay_{int(time.time())}.png'
            )
            cv2.imwrite(str(output), display)
            self.get_logger().info(f'Saved {output}')


def main():
    """Run the TF validation overlay."""
    rclpy.init()
    node = TfOverlayNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
