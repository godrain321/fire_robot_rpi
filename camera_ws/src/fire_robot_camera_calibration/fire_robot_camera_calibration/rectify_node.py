"""Rectify pinhole or equidistant camera images from camera_info YAML."""

from pathlib import Path

import cv2
from cv_bridge import CvBridge
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, CompressedImage, Image
import yaml


class RectifyCameraNode(Node):
    """Publish a rectified image and matching zero-distortion CameraInfo."""

    def __init__(self):
        super().__init__('rectify_camera')

        self.declare_parameter('input_topic', '/camera/image_raw')
        self.declare_parameter('input_transport', 'raw')
        self.declare_parameter('output_image_topic', '/camera/image_rect')
        self.declare_parameter(
            'output_camera_info_topic',
            '/camera/camera_info_rect',
        )
        self.declare_parameter('camera_info_path', '')
        self.declare_parameter('frame_id', 'camera_optical_frame')
        self.declare_parameter('balance', 0.3)

        self.input_topic = str(self.get_parameter('input_topic').value)
        self.input_transport = str(
            self.get_parameter('input_transport').value
        ).lower()
        self.output_image_topic = str(
            self.get_parameter('output_image_topic').value
        )
        self.output_camera_info_topic = str(
            self.get_parameter('output_camera_info_topic').value
        )
        camera_info_path = Path(
            str(self.get_parameter('camera_info_path').value)
        ).expanduser()
        self.frame_id = str(self.get_parameter('frame_id').value)
        self.balance = float(self.get_parameter('balance').value)

        if self.input_transport not in ('raw', 'compressed'):
            raise ValueError(
                "input_transport must be 'raw' or 'compressed'"
            )
        if not 0.0 <= self.balance <= 1.0:
            raise ValueError('balance must be between 0.0 and 1.0')
        if not camera_info_path.is_file():
            raise RuntimeError(
                f'camera_info_path does not exist: {camera_info_path}'
            )

        (
            self.camera_matrix,
            self.distortion,
            self.image_size,
            self.distortion_model,
        ) = self._load_camera_info(camera_info_path)
        self.new_camera_matrix, self.map_x, self.map_y = (
            self._build_rectification_maps()
        )
        self.bridge = CvBridge()
        self.bad_size_reported = False

        self.image_publisher = self.create_publisher(
            Image,
            self.output_image_topic,
            qos_profile_sensor_data,
        )
        self.info_publisher = self.create_publisher(
            CameraInfo,
            self.output_camera_info_topic,
            qos_profile_sensor_data,
        )

        message_type = (
            Image
            if self.input_transport == 'raw'
            else CompressedImage
        )
        self.subscription = self.create_subscription(
            message_type,
            self.input_topic,
            self._image_callback,
            qos_profile_sensor_data,
        )

        self.get_logger().info(
            f'Loaded {self.distortion_model} calibration from '
            f'{camera_info_path}'
        )
        self.get_logger().info(
            f'{self.input_topic} -> {self.output_image_topic}'
        )

    @staticmethod
    def _load_camera_info(path):
        with path.open('r', encoding='utf-8') as stream:
            data = yaml.safe_load(stream)

        required = (
            'image_width',
            'image_height',
            'camera_matrix',
            'distortion_coefficients',
            'distortion_model',
        )
        missing = [key for key in required if key not in data]
        if missing:
            raise ValueError(
                f'Camera YAML is missing: {", ".join(missing)}'
            )

        width = int(data['image_width'])
        height = int(data['image_height'])
        camera_matrix = np.asarray(
            data['camera_matrix']['data'],
            dtype=np.float64,
        ).reshape(3, 3)
        distortion = np.asarray(
            data['distortion_coefficients']['data'],
            dtype=np.float64,
        ).reshape(-1, 1)
        model = str(data['distortion_model'])
        return camera_matrix, distortion, (width, height), model

    def _build_rectification_maps(self):
        width, height = self.image_size
        identity = np.eye(3, dtype=np.float64)

        if self.distortion_model == 'equidistant':
            if self.distortion.size != 4:
                raise ValueError(
                    'equidistant calibration requires four coefficients'
                )
            new_camera_matrix = (
                cv2.fisheye.estimateNewCameraMatrixForUndistortRectify(
                    self.camera_matrix,
                    self.distortion,
                    self.image_size,
                    identity,
                    balance=self.balance,
                    new_size=self.image_size,
                    fov_scale=1.0,
                )
            )
            map_x, map_y = cv2.fisheye.initUndistortRectifyMap(
                self.camera_matrix,
                self.distortion,
                identity,
                new_camera_matrix,
                self.image_size,
                cv2.CV_16SC2,
            )
        elif self.distortion_model in (
            'plumb_bob',
            'rational_polynomial',
        ):
            new_camera_matrix, _ = cv2.getOptimalNewCameraMatrix(
                self.camera_matrix,
                self.distortion,
                (width, height),
                self.balance,
                (width, height),
            )
            map_x, map_y = cv2.initUndistortRectifyMap(
                self.camera_matrix,
                self.distortion,
                identity,
                new_camera_matrix,
                self.image_size,
                cv2.CV_16SC2,
            )
        else:
            raise ValueError(
                f'Unsupported distortion_model: {self.distortion_model}'
            )
        return new_camera_matrix, map_x, map_y

    def _decode(self, message):
        if self.input_transport == 'raw':
            return self.bridge.imgmsg_to_cv2(
                message,
                desired_encoding='bgr8',
            )
        encoded = np.frombuffer(message.data, dtype=np.uint8)
        return cv2.imdecode(encoded, cv2.IMREAD_COLOR)

    def _camera_info(self, stamp):
        width, height = self.image_size
        message = CameraInfo()
        message.header.stamp = stamp
        message.header.frame_id = self.frame_id
        message.width = width
        message.height = height
        message.distortion_model = 'plumb_bob'
        message.d = [0.0] * 5
        message.k = self.new_camera_matrix.reshape(-1).tolist()
        message.r = np.eye(3, dtype=np.float64).reshape(-1).tolist()

        projection = np.zeros((3, 4), dtype=np.float64)
        projection[:, :3] = self.new_camera_matrix
        message.p = projection.reshape(-1).tolist()
        return message

    def _image_callback(self, message):
        frame = self._decode(message)
        if frame is None:
            self.get_logger().warning('Image decode failed')
            return

        actual_size = (frame.shape[1], frame.shape[0])
        if actual_size != self.image_size:
            if not self.bad_size_reported:
                self.get_logger().error(
                    f'Image size {actual_size} does not match calibration '
                    f'{self.image_size}. Use the same camera mode used for '
                    'intrinsic calibration.'
                )
                self.bad_size_reported = True
            return

        rectified = cv2.remap(
            frame,
            self.map_x,
            self.map_y,
            interpolation=cv2.INTER_LINEAR,
            borderMode=cv2.BORDER_CONSTANT,
        )
        output = self.bridge.cv2_to_imgmsg(
            rectified,
            encoding='bgr8',
        )
        output.header.stamp = message.header.stamp
        output.header.frame_id = self.frame_id

        self.image_publisher.publish(output)
        self.info_publisher.publish(
            self._camera_info(message.header.stamp)
        )


def main():
    """Run the rectification node."""
    rclpy.init()
    node = RectifyCameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
