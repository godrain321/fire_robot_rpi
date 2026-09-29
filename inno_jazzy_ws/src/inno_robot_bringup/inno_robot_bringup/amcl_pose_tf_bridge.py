"""Continuously broadcast map->odom from the latest AMCL map pose.

AMCL on this platform stops refreshing its TF while the stationary RF2O pose is
below update thresholds. Navigation consumers require a continuously valid TF,
so this node holds the latest AMCL correction and republishes it at 20 Hz.
"""

import math

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from rclpy.node import Node
from tf2_ros import Buffer, TransformBroadcaster, TransformException, TransformListener


def yaw(q):
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


class AmclPoseTfBridge(Node):
    def __init__(self):
        super().__init__('amcl_pose_tf_bridge')
        self.declare_parameter('set_initial_pose', False)
        self.declare_parameter('initial_pose_x', 0.0)
        self.declare_parameter('initial_pose_y', 0.0)
        self.declare_parameter('initial_pose_yaw', 0.0)
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.broadcaster = TransformBroadcaster(self)
        self.map_to_odom = None
        self.initial_pose = None
        if bool(self.get_parameter('set_initial_pose').value):
            self.initial_pose = (
                float(self.get_parameter('initial_pose_x').value),
                float(self.get_parameter('initial_pose_y').value),
                float(self.get_parameter('initial_pose_yaw').value),
            )
            transform = TransformStamped()
            transform.header.frame_id = 'map'
            transform.child_frame_id = 'odom'
            transform.transform.translation.x = self.initial_pose[0]
            transform.transform.translation.y = self.initial_pose[1]
            transform.transform.rotation.z = math.sin(0.5 * self.initial_pose[2])
            transform.transform.rotation.w = math.cos(0.5 * self.initial_pose[2])
            self.map_to_odom = transform
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self._pose, 10)
        self.create_timer(0.05, self._broadcast)
        self.get_logger().info('Holding AMCL correction as continuous map->odom TF')

    def _pose(self, message):
        self._update_transform(
            message.pose.pose.position.x,
            message.pose.pose.position.y,
            yaw(message.pose.pose.orientation),
        )

    def _update_transform(self, map_base_x, map_base_y, map_base_yaw):
        try:
            odom_base = self.buffer.lookup_transform(
                'odom', 'base_link', rclpy.time.Time()
            )
        except TransformException as exc:
            self.get_logger().warning(
                f'Waiting for odom->base_link: {exc}', throttle_duration_sec=2.0
            )
            return
        odom_base_yaw = yaw(odom_base.transform.rotation)
        map_odom_yaw = math.atan2(
            math.sin(map_base_yaw - odom_base_yaw),
            math.cos(map_base_yaw - odom_base_yaw),
        )
        cos_yaw = math.cos(map_odom_yaw)
        sin_yaw = math.sin(map_odom_yaw)
        ox = odom_base.transform.translation.x
        oy = odom_base.transform.translation.y
        transform = TransformStamped()
        transform.header.frame_id = 'map'
        transform.child_frame_id = 'odom'
        transform.transform.translation.x = (
            map_base_x - (cos_yaw * ox - sin_yaw * oy)
        )
        transform.transform.translation.y = (
            map_base_y - (sin_yaw * ox + cos_yaw * oy)
        )
        transform.transform.rotation.z = math.sin(0.5 * map_odom_yaw)
        transform.transform.rotation.w = math.cos(0.5 * map_odom_yaw)
        self.map_to_odom = transform
        self.initial_pose = None
        self._broadcast()

    def _broadcast(self):
        if self.initial_pose is not None:
            self._update_transform(*self.initial_pose)
        if self.map_to_odom is None:
            return
        self.map_to_odom.header.stamp = self.get_clock().now().to_msg()
        self.broadcaster.sendTransform(self.map_to_odom)


def main(args=None):
    rclpy.init(args=args)
    node = AmclPoseTfBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
