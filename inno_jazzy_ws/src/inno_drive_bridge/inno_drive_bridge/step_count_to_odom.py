import math

import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from std_msgs.msg import Int64MultiArray
from tf2_ros import TransformBroadcaster


class StepCountToOdom(Node):
    def __init__(self):
        super().__init__('step_count_to_odom')
        defaults = {
            'wheel_radius': 0.04,
            'wheel_separation': 0.30,
            'motor_full_steps_per_rev': 200,
            'microsteps': 8,
            'gear_ratio': 1.0,
            'odom_frame': 'wheel_odom',
            'base_frame': 'base_link',
            'publish_tf': False,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

        self.wheel_radius = float(self.get_parameter('wheel_radius').value)
        self.wheel_separation = float(self.get_parameter('wheel_separation').value)
        self.full_steps = int(self.get_parameter('motor_full_steps_per_rev').value)
        self.microsteps = int(self.get_parameter('microsteps').value)
        self.gear_ratio = float(self.get_parameter('gear_ratio').value)
        self.odom_frame = str(self.get_parameter('odom_frame').value)
        self.base_frame = str(self.get_parameter('base_frame').value)
        self.publish_tf = bool(self.get_parameter('publish_tf').value)
        self._validate_parameters()

        steps_per_rev = self.full_steps * self.microsteps * self.gear_ratio
        self.meters_per_step = 2.0 * math.pi * self.wheel_radius / steps_per_rev
        self.odom_publisher = self.create_publisher(Odometry, '/wheel_odom', 10)
        self.path_publisher = self.create_publisher(Path, '/wheel_path', 10)
        self.create_subscription(
            Int64MultiArray, '/wheel_ticks', self._ticks_callback, 10
        )
        self.tf_broadcaster = TransformBroadcaster(self) if self.publish_tf else None

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.previous_left = None
        self.previous_right = None
        self.previous_time = None
        self.path = Path()
        self.path.header.frame_id = self.odom_frame
        self.get_logger().info(
            f'Step odometry ready; publish_tf={self.publish_tf}, '
            f'meters_per_step={self.meters_per_step:.8f}'
        )

    def _validate_parameters(self):
        if self.wheel_radius <= 0.0 or self.wheel_separation <= 0.0:
            raise ValueError('wheel_radius and wheel_separation must be greater than zero')
        if self.full_steps <= 0 or self.microsteps <= 0 or self.gear_ratio <= 0.0:
            raise ValueError('motor step parameters and gear_ratio must be greater than zero')
        if not self.odom_frame or not self.base_frame:
            raise ValueError('odom_frame and base_frame must not be empty')

    def _ticks_callback(self, message):
        if len(message.data) < 2:
            self.get_logger().warning('/wheel_ticks must contain [left_count, right_count]')
            return

        left_count = int(message.data[0])
        right_count = int(message.data[1])
        now = self.get_clock().now()
        if self.previous_left is None:
            self.previous_left = left_count
            self.previous_right = right_count
            self.previous_time = now
            return

        delta_left = (left_count - self.previous_left) * self.meters_per_step
        delta_right = (right_count - self.previous_right) * self.meters_per_step
        dt = (now - self.previous_time).nanoseconds * 1e-9
        self.previous_left = left_count
        self.previous_right = right_count
        self.previous_time = now

        delta_distance = 0.5 * (delta_right + delta_left)
        delta_yaw = (delta_right - delta_left) / self.wheel_separation
        midpoint_yaw = self.yaw + 0.5 * delta_yaw
        self.x += delta_distance * math.cos(midpoint_yaw)
        self.y += delta_distance * math.sin(midpoint_yaw)
        self.yaw = math.atan2(
            math.sin(self.yaw + delta_yaw), math.cos(self.yaw + delta_yaw)
        )

        linear_velocity = delta_distance / dt if dt > 0.0 else 0.0
        angular_velocity = delta_yaw / dt if dt > 0.0 else 0.0
        self._publish(now, linear_velocity, angular_velocity)

    def _publish(self, now, linear_velocity, angular_velocity):
        stamp = now.to_msg()
        half_yaw = 0.5 * self.yaw
        orientation_z = math.sin(half_yaw)
        orientation_w = math.cos(half_yaw)

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.z = orientation_z
        odom.pose.pose.orientation.w = orientation_w
        odom.twist.twist.linear.x = linear_velocity
        odom.twist.twist.angular.z = angular_velocity
        self.odom_publisher.publish(odom)

        pose = PoseStamped()
        pose.header = odom.header
        pose.pose = odom.pose.pose
        self.path.header.stamp = stamp
        self.path.poses.append(pose)
        if len(self.path.poses) > 10000:
            self.path.poses = self.path.poses[-10000:]
        self.path_publisher.publish(self.path)

        if self.tf_broadcaster is not None:
            transform = TransformStamped()
            transform.header = odom.header
            transform.child_frame_id = self.base_frame
            transform.transform.translation.x = self.x
            transform.transform.translation.y = self.y
            transform.transform.rotation.z = orientation_z
            transform.transform.rotation.w = orientation_w
            self.tf_broadcaster.sendTransform(transform)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = StepCountToOdom()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except ValueError as error:
        if node is not None:
            node.get_logger().error(str(error))
        else:
            print(f'step_count_to_odom: {error}')
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
