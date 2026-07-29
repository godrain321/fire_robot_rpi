"""CLI publisher for semantic navigation missions."""

import argparse
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


def parser() -> argparse.ArgumentParser:
    result = argparse.ArgumentParser(
        description='목적지 또는 source/destination semantic mission을 발행합니다.'
    )
    result.add_argument('labels', nargs='+', help='exit2 또는 exit1 exit2')
    result.add_argument('--wait-sec', type=float, default=1.0, help='DDS discovery 대기시간')
    return result


def main(args=None) -> None:
    parsed, ros_args = parser().parse_known_args(args)
    if len(parsed.labels) not in (1, 2):
        parser().error('label은 목적지 1개 또는 source/destination 2개여야 합니다.')
    if parsed.wait_sec < 0.0:
        parser().error('--wait-sec는 0 이상이어야 합니다.')
    mission = 'go ' + ' '.join(parsed.labels)
    rclpy.init(args=ros_args)
    node = Node('go_to')
    publisher = node.create_publisher(String, '/mission_text', 10)
    deadline = time.monotonic() + parsed.wait_sec
    try:
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
        message = String(data=mission)
        for _ in range(3):
            publisher.publish(message)
            rclpy.spin_once(node, timeout_sec=0.10)
        print(f'/mission_text 발행: {mission}')
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
