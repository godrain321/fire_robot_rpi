"""Publish the observed robot trail from TF, independent of wheel encoders."""

from collections import deque
import math

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from rclpy.node import Node
from tf2_ros import Buffer, TransformException, TransformListener


class TfToPath(Node):
    def __init__(self):
        super().__init__('tf_to_path')
        for name, value in (
            ('fixed_frame', 'map'), ('base_frame', 'base_link'),
            ('path_topic', '/lidar_path'), ('publish_rate_hz', 10.0),
            ('minimum_spacing', 0.02), ('max_points', 10000),
        ):
            self.declare_parameter(name, value)
        self.fixed = str(self.get_parameter('fixed_frame').value)
        self.base = str(self.get_parameter('base_frame').value)
        self.spacing = float(self.get_parameter('minimum_spacing').value)
        rate = float(self.get_parameter('publish_rate_hz').value)
        self.poses = deque(maxlen=int(self.get_parameter('max_points').value))
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.publisher = self.create_publisher(
            Path, str(self.get_parameter('path_topic').value), 10
        )
        self.create_timer(1.0 / rate, self._sample)

    def _sample(self):
        try:
            transform = self.buffer.lookup_transform(
                self.fixed, self.base, rclpy.time.Time()
            )
        except TransformException:
            return
        translation = transform.transform.translation
        rotation = transform.transform.rotation
        values = (translation.x, translation.y, rotation.x, rotation.y, rotation.z, rotation.w)
        if not all(math.isfinite(value) for value in values):
            return
        if self.poses:
            previous = self.poses[-1].pose.position
            if math.hypot(translation.x - previous.x, translation.y - previous.y) < self.spacing:
                return
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = self.fixed
        pose.pose.position.x = translation.x
        pose.pose.position.y = translation.y
        pose.pose.position.z = translation.z
        pose.pose.orientation = rotation
        self.poses.append(pose)
        path = Path()
        path.header = pose.header
        path.poses = list(self.poses)
        self.publisher.publish(path)


def main(args=None):
    rclpy.init(args=args)
    node = TfToPath()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
