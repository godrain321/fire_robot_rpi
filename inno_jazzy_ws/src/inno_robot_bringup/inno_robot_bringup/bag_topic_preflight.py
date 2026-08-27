"""Report live publisher/sample status without blocking rosbag recording."""

import argparse
import time
from typing import Dict, List, Set

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rosidl_runtime_py.utilities import get_message


def _publisher_names(endpoints) -> str:
    names = sorted({endpoint.node_name for endpoint in endpoints})
    return ','.join(names) if names else '-'


def main(args=None) -> None:
    parser = argparse.ArgumentParser(
        description='Check publishers and real samples for rosbag topics.'
    )
    parser.add_argument(
        '--wait', type=float, default=2.0,
        help='seconds to wait for discovery and samples (default: 2.0)',
    )
    parser.add_argument('topics', nargs='+')
    parsed, ros_args = parser.parse_known_args(args)
    if parsed.wait <= 0.0:
        parser.error('--wait must be positive')

    rclpy.init(args=ros_args)
    node = Node('robot_bag_topic_preflight')
    subscriptions: List[object] = []
    received: Set[str] = set()
    publishers: Dict[str, list] = {}
    type_by_topic: Dict[str, str] = {}
    probe_errors: Dict[str, str] = {}
    # A best-effort/volatile request is compatible with both reliable and
    # best-effort publishers.  Do not copy endpoint QoS verbatim: graph
    # discovery can legitimately report UNKNOWN policies (notably for this
    # node's own /rosout and /parameter_events endpoints), which rclpy rejects
    # when used to construct a subscription.
    probe_qos = QoSProfile(depth=10)
    probe_qos.reliability = ReliabilityPolicy.BEST_EFFORT
    probe_qos.durability = DurabilityPolicy.VOLATILE
    try:
        discovery_deadline = time.monotonic() + min(1.0, parsed.wait * 0.5)
        while time.monotonic() < discovery_deadline:
            rclpy.spin_once(node, timeout_sec=0.05)

        topic_types = dict(node.get_topic_names_and_types())
        for topic in parsed.topics:
            endpoints = node.get_publishers_info_by_topic(topic)
            publishers[topic] = endpoints
            types = topic_types.get(topic, [])
            if not endpoints or not types:
                continue
            type_name = types[0]
            type_by_topic[topic] = type_name
            try:
                message_type = get_message(type_name)
                subscription = node.create_subscription(
                    message_type,
                    topic,
                    lambda _message, name=topic: received.add(name),
                    probe_qos,
                )
                subscriptions.append(subscription)
            except (
                AttributeError, ImportError, LookupError, RuntimeError
            ) as error:
                probe_errors[topic] = str(error)

        sample_deadline = time.monotonic() + parsed.wait
        while time.monotonic() < sample_deadline:
            rclpy.spin_once(node, timeout_sec=0.05)

        print(
            f'=== ROS topic preflight '
            f'({parsed.wait:.1f}s sample window) ==='
        )
        counts = {'received': 0, 'waiting': 0, 'missing': 0}
        for topic in parsed.topics:
            endpoints = publishers.get(topic, [])
            if not endpoints:
                counts['missing'] += 1
                print(
                    f'[없음] {topic} | 발행자 없음 '
                    '(그래도 녹화 대상 유지)'
                )
            elif topic in received:
                counts['received'] += 1
                print(
                    f'[수신] {topic} | {type_by_topic.get(topic, "unknown")} | '
                    f'발행자={_publisher_names(endpoints)}'
                )
            else:
                counts['waiting'] += 1
                detail = probe_errors.get(
                    topic,
                    f'{parsed.wait:.1f}s 동안 새 메시지 없음',
                )
                print(
                    f'[대기] {topic} | 발행자={_publisher_names(endpoints)} | '
                    f'{detail} (그래도 녹화 대상 유지)'
                )
        print(
            '요약: '
            f'수신 {counts["received"]}, '
            f'발행자만 있음 {counts["waiting"]}, '
            f'발행자 없음 {counts["missing"]}'
        )
    finally:
        del subscriptions
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
