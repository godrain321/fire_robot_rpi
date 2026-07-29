"""Publish the no-go-composited planning map as a transient OccupancyGrid."""

import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from .grid_utils import GridError, load_pgm_as_occupancy, quaternion_from_yaw


class PlanningGridPublisher(Node):
    def __init__(self) -> None:
        super().__init__('planning_grid_publisher')
        self.declare_parameter(
            'map_yaml', '/home/gosunwoo/fire_robot_rpi/maps/inno_map_nav.yaml'
        )
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('publish_rate_hz', 1.0)
        map_yaml = str(self.get_parameter('map_yaml').value)
        map_frame = str(self.get_parameter('map_frame').value)
        publish_rate = float(self.get_parameter('publish_rate_hz').value)
        if publish_rate <= 0.0:
            raise GridError('publish_rate_hz는 0보다 커야 합니다.')
        self.grid = load_pgm_as_occupancy(map_yaml, map_frame)
        qos = QoSProfile(depth=1)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.publisher = self.create_publisher(
            OccupancyGrid, '/planning_grid_static', qos
        )
        self.timer = self.create_timer(1.0 / publish_rate, self.publish_grid)
        self.publish_grid()
        self.get_logger().info(
            f'planning grid: {self.grid.width}x{self.grid.height}, '
            f'{self.grid.resolution:.3f} m/cell, source={map_yaml}'
        )

    def publish_grid(self) -> None:
        message = OccupancyGrid()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self.grid.frame_id
        message.info.map_load_time = message.header.stamp
        message.info.resolution = self.grid.resolution
        message.info.width = self.grid.width
        message.info.height = self.grid.height
        message.info.origin.position.x = self.grid.origin_x
        message.info.origin.position.y = self.grid.origin_y
        qx, qy, qz, qw = quaternion_from_yaw(self.grid.origin_yaw)
        message.info.origin.orientation.x = qx
        message.info.origin.orientation.y = qy
        message.info.origin.orientation.z = qz
        message.info.origin.orientation.w = qw
        message.data = self.grid.data.reshape(-1).astype(int).tolist()
        self.publisher.publish(message)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = PlanningGridPublisher()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except GridError as exc:
        print(f'planning_grid_publisher 오류: {exc}')
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
