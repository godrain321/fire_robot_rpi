"""Show one short robot-heading arrow from the live map->base_link TF."""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Int32
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray


def marker_from_transform(transform, *, length_m=0.55, width_m=0.12):
    """Build the one stable-ID marker used to represent the robot heading."""

    translation = transform.transform.translation
    rotation = transform.transform.rotation
    values = (
        translation.x, translation.y, translation.z,
        rotation.x, rotation.y, rotation.z, rotation.w,
        length_m, width_m,
    )
    if not all(math.isfinite(float(value)) for value in values):
        raise ValueError('TF and marker dimensions must be finite')
    if length_m <= 0.0 or width_m <= 0.0:
        raise ValueError('marker dimensions must be positive')

    marker = Marker()
    marker.header = transform.header
    marker.ns = 'robot_heading_tf'
    marker.id = 0
    marker.type = Marker.ARROW
    marker.action = Marker.ADD
    marker.pose.position.x = translation.x
    marker.pose.position.y = translation.y
    marker.pose.position.z = translation.z + 0.10
    marker.pose.orientation = rotation
    marker.scale.x = float(length_m)
    marker.scale.y = float(width_m)
    marker.scale.z = float(width_m)
    marker.color.r = 1.0
    marker.color.g = 0.08
    marker.color.b = 0.08
    marker.color.a = 0.95
    return marker


def delete_heading_marker(frame_id, stamp):
    """Delete the stable heading marker outside MODE 3."""

    marker = Marker()
    marker.header.frame_id = frame_id
    marker.header.stamp = stamp
    marker.ns = 'robot_heading_tf'
    marker.id = 0
    marker.action = Marker.DELETE
    return marker


class TfHeadingMarker(Node):
    """Continuously replace one marker with the latest robot TF pose."""

    def __init__(self):
        super().__init__('tf_heading_marker')
        defaults = {
            'fixed_frame': 'map',
            'base_frame': 'base_link',
            'marker_topic': '/robot_heading_marker',
            'publish_rate_hz': 15.0,
            'arrow_length_m': 0.55,
            'arrow_width_m': 0.12,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

        self.fixed_frame = str(self.get_parameter('fixed_frame').value)
        self.base_frame = str(self.get_parameter('base_frame').value)
        self.arrow_length = float(
            self.get_parameter('arrow_length_m').value
        )
        self.arrow_width = float(
            self.get_parameter('arrow_width_m').value
        )
        rate = float(self.get_parameter('publish_rate_hz').value)
        if not self.fixed_frame or not self.base_frame:
            raise ValueError('TF frame names must not be empty')
        if rate <= 0.0:
            raise ValueError('publish_rate_hz must be positive')
        if self.arrow_length <= 0.0 or self.arrow_width <= 0.0:
            raise ValueError('arrow dimensions must be positive')

        qos = QoSProfile(depth=1)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.publisher = self.create_publisher(
            MarkerArray,
            str(self.get_parameter('marker_topic').value),
            qos,
        )
        self.mode3_enabled = False
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.create_subscription(Int32, '/drive_mode', self._mode_callback, 10)
        self.create_timer(1.0 / rate, self._publish_marker)
        self.get_logger().info(
            f'Single TF heading marker: {self.fixed_frame} -> '
            f'{self.base_frame}, length={self.arrow_length:.2f}m'
        )

    def _mode_callback(self, message: Int32) -> None:
        enabled = int(message.data) == 3
        if enabled == self.mode3_enabled:
            return
        self.mode3_enabled = enabled
        if not enabled:
            marker = delete_heading_marker(
                self.fixed_frame, self.get_clock().now().to_msg()
            )
            self.publisher.publish(MarkerArray(markers=[marker]))

    def _publish_marker(self):
        if not self.mode3_enabled:
            return
        try:
            transform = self.buffer.lookup_transform(
                self.fixed_frame, self.base_frame, rclpy.time.Time()
            )
        except TransformException:
            return
        try:
            marker = marker_from_transform(
                transform,
                length_m=self.arrow_length,
                width_m=self.arrow_width,
            )
        except ValueError:
            return
        marker.header.stamp = self.get_clock().now().to_msg()
        self.publisher.publish(MarkerArray(markers=[marker]))


def main(args=None):
    rclpy.init(args=args)
    node = TfHeadingMarker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
