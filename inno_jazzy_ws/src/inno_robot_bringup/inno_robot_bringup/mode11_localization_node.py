"""Mode 11 scan gate and sole map/odom/base_link TF selector."""

import math
import time

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, LaserScan
from std_msgs.msg import Empty, Int64MultiArray, String
from tf2_ros import TransformBroadcaster
from visualization_msgs.msg import Marker, MarkerArray

from .mode11_fallback_core import (
    EncoderImuFallback, Mode11State, Pose2D, normalize_yaw,
)


def yaw_from_quaternion(q):
    components = (q.x, q.y, q.z, q.w)
    if not all(math.isfinite(v) for v in components):
        return None
    norm = math.sqrt(sum(v * v for v in components))
    if not 0.5 <= norm <= 1.5:
        return None
    x, y, z, w = (v / norm for v in components)
    return math.atan2(2.0 * (w * z + x * y),
                      1.0 - 2.0 * (y * y + z * z))


def fill_yaw(q, yaw):
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw * 0.5)
    q.w = math.cos(yaw * 0.5)


class Mode11Localization(Node):
    def __init__(self):
        super().__init__('mode11_localization')
        defaults = {
            'imu_topic': '/imu/data_raw',
            'encoder_topic': '/wheel_encoder_ticks',
            'allow_virtual_step_counts': False,
            'imu_yaw_sign': 1,
            'require_map_alignment': True,
            'rf2o_timeout_sec': 0.5,
            'encoder_timeout_sec': 0.6,
            'imu_timeout_sec': 0.35,
            'scan_timeout_sec': 0.5,
            'wheel_radius': 0.04,
            'motor_full_steps_per_rev': 200,
            'microsteps': 8,
            'gear_ratio': 1.0,
            'left_sign': -1,
            'right_sign': 1,
            'maximum_steps_per_sec': 2400.0,
        }
        for name, default in defaults.items():
            self.declare_parameter(name, default)
        value = lambda name: self.get_parameter(name).value
        self.imu_topic = str(value('imu_topic'))
        self.encoder_topic = str(value('encoder_topic'))
        self.allow_virtual_step_counts = bool(value('allow_virtual_step_counts'))
        self.imu_yaw_sign = int(value('imu_yaw_sign'))
        self.require_map_alignment = bool(value('require_map_alignment'))
        if self.imu_yaw_sign not in (-1, 1):
            raise ValueError('imu_yaw_sign must be -1 or 1')
        if not self.encoder_topic:
            raise ValueError('encoder_topic must name a std_msgs/Int64MultiArray topic')
        self.rf2o_timeout = float(value('rf2o_timeout_sec'))
        self.encoder_timeout = float(value('encoder_timeout_sec'))
        self.imu_timeout = float(value('imu_timeout_sec'))
        self.scan_timeout = float(value('scan_timeout_sec'))
        if min(self.rf2o_timeout, self.encoder_timeout,
               self.imu_timeout, self.scan_timeout) <= 0:
            raise ValueError('sensor timeouts must be positive')
        self.fallback = EncoderImuFallback(
            wheel_radius=float(value('wheel_radius')),
            full_steps=int(value('motor_full_steps_per_rev')),
            microsteps=int(value('microsteps')),
            gear_ratio=float(value('gear_ratio')),
            left_sign=int(value('left_sign')),
            right_sign=int(value('right_sign')),
            maximum_steps_per_sec=float(value('maximum_steps_per_sec')),
        )
        self.state = Mode11State.NORMAL
        self.rf2o_pose = None
        self.rf2o_message = None
        self.rf2o_receipt = None
        self.ticks = None
        self.ticks_receipt = None
        self.tick_samples = 0
        self.imu_stamp = None
        self.imu_receipt = None
        self.imu_angular_z = None
        self.imu_interval_ok = False
        self.map_to_odom = None
        self.last_sensor_warning = None
        self.rf2o_stale_warning = False

        self.scan_publisher = self.create_publisher(
            LaserScan, '/mode11/scan', qos_profile_sensor_data
        )
        self.odom_publisher = self.create_publisher(
            Odometry, '/odom_selected', 10
        )
        self.state_publisher = self.create_publisher(
            String, '/mode11/state', 10
        )
        self.body_publisher = self.create_publisher(
            MarkerArray, '/mode11/robot_body', 10
        )
        self.tf_broadcaster = TransformBroadcaster(self)
        self.create_subscription(
            LaserScan, '/scan', self._scan, qos_profile_sensor_data
        )
        self.create_subscription(Odometry, '/odom_rf2o', self._rf2o, 10)
        self.create_subscription(
            Int64MultiArray, self.encoder_topic, self._encoder, 10
        )
        if self.imu_topic:
            self.create_subscription(Imu, self.imu_topic, self._imu, 10)
        self.create_subscription(
            PoseWithCovarianceStamped, '/amcl_pose', self._amcl_pose, 10
        )
        self.create_subscription(
            Empty, '/mode11/blackout_request', self._blackout_request, 10
        )
        self.create_timer(0.05, self._publish)
        self.state_publisher.publish(String(data=self.state.value))
        self.get_logger().info('[MODE11] State: NORMAL - RF2O localization')
        if not self.imu_topic:
            self.get_logger().warning(
                '[MODE11] IMU topic is not configured; P will be rejected. '
                'Set imu_topic to the actual sensor_msgs/Imu topic.'
            )
        if (self.encoder_topic == '/wheel_ticks'
                and not self.allow_virtual_step_counts):
            self.get_logger().warning(
                '[MODE11] /wheel_ticks contains commanded STEP counts, not '
                'physical wheel encoder readings. P will be rejected unless '
                'a physical encoder_topic is set or '
                'allow_virtual_step_counts:=true is explicitly selected.'
            )

    def _ros_seconds(self):
        return self.get_clock().now().nanoseconds * 1e-9

    @staticmethod
    def _stamp_seconds(header):
        return header.stamp.sec + header.stamp.nanosec * 1e-9

    def _stamp_fresh(self, header, timeout):
        stamp = self._stamp_seconds(header)
        age = self._ros_seconds() - stamp
        return stamp > 0.0 and -0.1 <= age <= timeout

    def _scan(self, message):
        if (self.state is Mode11State.NORMAL
                and self._stamp_fresh(message.header, self.scan_timeout)):
            self.scan_publisher.publish(message)

    def _rf2o(self, message):
        if self.state is Mode11State.BLACKOUT:
            return
        if (message.header.frame_id != 'odom'
                or message.child_frame_id != 'base_link'
                or not self._stamp_fresh(message.header, self.rf2o_timeout)):
            return
        point = message.pose.pose.position
        yaw = yaw_from_quaternion(message.pose.pose.orientation)
        if yaw is None or not all(math.isfinite(v) for v in (point.x, point.y)):
            return
        self.rf2o_pose = Pose2D(point.x, point.y, yaw)
        self.rf2o_message = message
        self.rf2o_receipt = time.monotonic()

    def _encoder(self, message):
        if len(message.data) < 2:
            return
        now = time.monotonic()
        self.ticks = (int(message.data[0]), int(message.data[1]))
        self.ticks_receipt = now
        self.tick_samples += 1
        if self.state is Mode11State.BLACKOUT:
            self.fallback.on_ticks(
                self.ticks, now, self._imu_fresh(now)
            )

    def _imu(self, message):
        angular_z = float(message.angular_velocity.z) * self.imu_yaw_sign
        if (message.angular_velocity_covariance[0] == -1.0
                or not math.isfinite(angular_z)
                or not self._stamp_fresh(message.header, self.imu_timeout)):
            return
        stamp = self._stamp_seconds(message.header)
        if self.imu_stamp is not None and stamp <= self.imu_stamp:
            return
        previous_stamp = self.imu_stamp
        self.imu_interval_ok = (previous_stamp is not None
                                and 0.0 < stamp - previous_stamp <= 0.25)
        self.imu_stamp = stamp
        self.imu_receipt = time.monotonic()
        self.imu_angular_z = angular_z
        if self.state is Mode11State.BLACKOUT:
            encoder_fresh = (self.ticks_receipt is not None
                             and time.monotonic() - self.ticks_receipt
                             <= self.encoder_timeout)
            if not self.fallback.on_imu(
                    angular_z, stamp, encoder_fresh=encoder_fresh):
                self.imu_interval_ok = False

    def _imu_fresh(self, now):
        return (self.imu_receipt is not None
                and self.imu_interval_ok
                and now - self.imu_receipt <= self.imu_timeout
                and self.imu_stamp is not None
                and 0.0 <= self._ros_seconds() - self.imu_stamp <= self.imu_timeout)

    def _amcl_pose(self, message):
        if self.state is Mode11State.BLACKOUT or self.rf2o_pose is None:
            return
        if not self._stamp_fresh(message.header, 1.0):
            return
        pose = message.pose.pose
        map_yaw = yaw_from_quaternion(pose.orientation)
        if (map_yaw is None or
                not all(math.isfinite(v) for v in
                        (pose.position.x, pose.position.y))):
            return
        odom = self.rf2o_pose
        yaw = normalize_yaw(map_yaw - odom.yaw)
        c, s = math.cos(yaw), math.sin(yaw)
        self.map_to_odom = Pose2D(
            pose.position.x - (c * odom.x - s * odom.y),
            pose.position.y - (s * odom.x + c * odom.y),
            yaw,
        )

    def _blackout_request(self, _message):
        if self.state is Mode11State.BLACKOUT:
            self.get_logger().info('[MODE11] Already in BLACKOUT')
            return
        now = time.monotonic()
        missing = []
        if (self.rf2o_pose is None or self.rf2o_receipt is None
                or now - self.rf2o_receipt > self.rf2o_timeout
                or not self._stamp_fresh(
                    self.rf2o_message.header, self.rf2o_timeout
                )):
            missing.append('RF2O pose')
        if (self.ticks is None or self.tick_samples < 2
                or now - self.ticks_receipt > self.encoder_timeout):
            missing.append(f'Encoder {self.encoder_topic}')
        if (self.encoder_topic == '/wheel_ticks'
                and not self.allow_virtual_step_counts):
            missing.append('physical Encoder (current /wheel_ticks is virtual STEP)')
        if not self.imu_topic:
            missing.append('IMU topic configuration')
        elif not self._imu_fresh(now):
            missing.append(f'IMU {self.imu_topic}')
        if self.require_map_alignment and self.map_to_odom is None:
            missing.append('AMCL map->odom')
        if missing:
            self.get_logger().error(
                '[MODE11] BLACKOUT refused; unavailable or stale: '
                + ', '.join(missing)
            )
            return
        pose = self.fallback.start(
            self.rf2o_pose, self.ticks, self.ticks_receipt, self.imu_stamp
        )
        self.state = Mode11State.BLACKOUT
        self.state_publisher.publish(String(data=self.state.value))
        self.get_logger().warning(
            '\n==================================================\n'
            'LiDAR 작동 멈춤 - Encoder + IMU 위치추정 전환\n'
            '=================================================='
        )
        self.get_logger().info('[MODE11] State: BLACKOUT')
        self.get_logger().info(
            f'[MODE11] fallback start pose: x={pose.x:.3f}, '
            f'y={pose.y:.3f}, yaw={math.degrees(pose.yaw):.2f} deg'
        )
        self._publish()

    def _publish(self):
        pose = (self.rf2o_pose if self.state is Mode11State.NORMAL
                else self.fallback.pose)
        if pose is None:
            return
        now = time.monotonic()
        if self.state is Mode11State.NORMAL:
            fresh = (self.rf2o_receipt is not None
                     and now - self.rf2o_receipt <= self.rf2o_timeout
                     and self._stamp_fresh(
                         self.rf2o_message.header, self.rf2o_timeout
                     ))
            if not fresh:
                if not self.rf2o_stale_warning:
                    self.get_logger().error(
                        '[MODE11] RF2O stale; selected odometry/TF paused'
                    )
                self.rf2o_stale_warning = True
                return
            if self.rf2o_stale_warning:
                self.get_logger().info('[MODE11] RF2O recovered')
            self.rf2o_stale_warning = False
        if self.state is Mode11State.BLACKOUT:
            missing = []
            if self.ticks_receipt is None or now - self.ticks_receipt > self.encoder_timeout:
                missing.append('Encoder')
            if not self._imu_fresh(now):
                missing.append('IMU')
            status = ', '.join(missing) if missing else None
            if status and status != self.last_sensor_warning:
                self.get_logger().error(
                    f'[MODE11] fallback sensor stale: {status}; pose held'
                )
            self.last_sensor_warning = status
        stamp = self.get_clock().now().to_msg()
        selected = Odometry()
        selected.header.stamp = stamp
        selected.header.frame_id = 'odom'
        selected.child_frame_id = 'base_link'
        selected.pose.pose.position.x = pose.x
        selected.pose.pose.position.y = pose.y
        fill_yaw(selected.pose.pose.orientation, pose.yaw)
        if self.state is Mode11State.NORMAL and self.rf2o_message is not None:
            selected.twist = self.rf2o_message.twist
        elif self.last_sensor_warning is None:
            selected.twist.twist.linear.x = self.fallback.linear_speed
            selected.twist.twist.angular.z = self.fallback.angular_speed
        self.odom_publisher.publish(selected)
        odom_tf = TransformStamped()
        odom_tf.header = selected.header
        odom_tf.child_frame_id = 'base_link'
        odom_tf.transform.translation.x = pose.x
        odom_tf.transform.translation.y = pose.y
        fill_yaw(odom_tf.transform.rotation, pose.yaw)
        self.tf_broadcaster.sendTransform(odom_tf)
        body = Marker()
        body.header.stamp = stamp
        body.header.frame_id = 'base_link'
        body.ns = 'mode11_schematic_body'
        body.id = 0
        body.type = Marker.CUBE
        body.action = Marker.ADD
        body.pose.orientation.w = 1.0
        body.scale.x = 0.34
        body.scale.y = 0.30
        body.scale.z = 0.08
        body.color.a = 0.65
        body.color.g = 0.55
        body.color.b = 1.0
        body.frame_locked = True
        front = Marker()
        front.header = body.header
        front.ns = body.ns
        front.id = 1
        front.type = Marker.ARROW
        front.action = Marker.ADD
        front.pose.position.x = 0.22
        front.pose.orientation.w = 1.0
        front.scale.x = 0.18
        front.scale.y = 0.08
        front.scale.z = 0.08
        front.color.a = 1.0
        front.color.r = 1.0
        front.color.g = 0.7
        front.frame_locked = True
        self.body_publisher.publish(MarkerArray(markers=[body, front]))
        if self.map_to_odom is not None:
            map_tf = TransformStamped()
            map_tf.header.stamp = stamp
            map_tf.header.frame_id = 'map'
            map_tf.child_frame_id = 'odom'
            map_tf.transform.translation.x = self.map_to_odom.x
            map_tf.transform.translation.y = self.map_to_odom.y
            fill_yaw(map_tf.transform.rotation, self.map_to_odom.yaw)
            self.tf_broadcaster.sendTransform(map_tf)


def main(args=None):
    rclpy.init(args=args)
    node = Mode11Localization()
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
