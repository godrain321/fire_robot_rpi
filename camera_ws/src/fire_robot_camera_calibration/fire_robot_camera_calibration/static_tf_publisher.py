"""Publish the calibrated LiDAR-to-camera transform as static TF."""

from pathlib import Path

from geometry_msgs.msg import TransformStamped
import numpy as np
import rclpy
from rclpy.node import Node
from tf2_ros import StaticTransformBroadcaster
import yaml

from .calibration_math import quaternion_from_rotation


class StaticTfPublisher(Node):
    """Load T_camera_lidar from YAML and publish it on /tf_static."""

    def __init__(self):
        super().__init__('lidar_camera_static_tf')

        self.declare_parameter('extrinsic_path', '')
        self.declare_parameter('camera_frame', '')
        self.declare_parameter('lidar_frame', '')

        extrinsic_path = Path(
            str(self.get_parameter('extrinsic_path').value)
        ).expanduser()
        camera_frame_override = str(
            self.get_parameter('camera_frame').value
        )
        lidar_frame_override = str(
            self.get_parameter('lidar_frame').value
        )

        if not extrinsic_path.is_file():
            raise RuntimeError(
                f'extrinsic_path does not exist: {extrinsic_path}'
            )
        with extrinsic_path.open('r', encoding='utf-8') as stream:
            data = yaml.safe_load(stream)

        try:
            transform = data['T_camera_lidar']
            rotation = np.asarray(
                transform['R_row_major'],
                dtype=np.float64,
            ).reshape(3, 3)
            translation = np.asarray(
                transform['t_xyz'],
                dtype=np.float64,
            ).reshape(3)
        except (KeyError, TypeError, ValueError) as error:
            raise ValueError(
                f'Invalid extrinsic YAML: {extrinsic_path}'
            ) from error

        camera_frame = (
            camera_frame_override
            or str(data.get('camera_frame', 'camera_optical_frame'))
        )
        lidar_frame = (
            lidar_frame_override
            or str(data.get('lidar_frame', 'laser_frame'))
        )
        quaternion = quaternion_from_rotation(rotation)

        message = TransformStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = camera_frame
        message.child_frame_id = lidar_frame
        message.transform.translation.x = float(translation[0])
        message.transform.translation.y = float(translation[1])
        message.transform.translation.z = float(translation[2])
        message.transform.rotation.x = float(quaternion[0])
        message.transform.rotation.y = float(quaternion[1])
        message.transform.rotation.z = float(quaternion[2])
        message.transform.rotation.w = float(quaternion[3])

        self.broadcaster = StaticTransformBroadcaster(self)
        self.broadcaster.sendTransform(message)
        self.get_logger().info(
            f'Published static TF {camera_frame} -> {lidar_frame} '
            f'from {extrinsic_path}'
        )


def main():
    """Run the static transform publisher."""
    rclpy.init()
    node = StaticTfPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
