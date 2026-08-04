#!/usr/bin/env python3
"""Capture odometry-based waypoints while driving with the keyboard.

Workflow:
1. Run the drive_keyboard_demo.launch.py (keyboard + odom + path output).
2. Run this node.
3. While driving, press `c` to capture the current robot pose as the next waypoint
   (`p1`, `p2`, `p3`, ...).
4. The pose is written into the semantic YAML file in the map frame (if TF is available).
"""

from __future__ import annotations

import argparse
import math
import select
import sys
import termios
import time
import tty
from pathlib import Path
from typing import Optional

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener
from tf2_geometry_msgs import do_transform_pose

from .geometry_utils import yaw_from_quaternion
from .semantic_store import SemanticStore, SemanticStoreError, default_document


class PathWaypointRecorder(Node):
    def __init__(self) -> None:
        super().__init__('path_waypoint_recorder')
        self.declare_parameter('semantic_file', '/home/gosunwoo/fire_robot_rpi/maps/semantic_points.yaml')
        self.declare_parameter('prefix', 'p')
        self.declare_parameter('start_index', 1)
        self.declare_parameter('odom_topic', '/wheel_odom')
        self.declare_parameter('target_frame', 'map')

        self.semantic_file = Path(str(self.get_parameter('semantic_file').value)).expanduser()
        self.prefix = str(self.get_parameter('prefix').value)
        self.start_index = int(self.get_parameter('start_index').value)
        self.odom_topic = str(self.get_parameter('odom_topic').value)
        self.target_frame = str(self.get_parameter('target_frame').value)

        self._next_index = self.start_index
        self._latest_pose: Optional[PoseStamped] = None
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self._owns_input_stream = False
        if sys.stdin.isatty():
            self._input_stream = sys.stdin
        else:
            try:
                self._input_stream = open('/dev/tty', encoding='utf-8')
                self._owns_input_stream = True
            except OSError as exc:
                raise RuntimeError('keyboard input requires an interactive terminal (TTY)') from exc

        self._terminal_settings = termios.tcgetattr(self._input_stream)
        tty.setcbreak(self._input_stream.fileno())

        self.create_subscription(Odometry, self.odom_topic, self._odom_cb, 10)
        self.create_timer(0.05, self._poll_keyboard)

        self.get_logger().info(
            f'Path waypoint recorder ready: semantic_file={self.semantic_file}, '
            f'prefix={self.prefix}, target_frame={self.target_frame}, '
            f'press c to capture next waypoint, q to quit'
        )

    def _odom_cb(self, msg: Odometry) -> None:
        pose = PoseStamped()
        pose.header = msg.header
        pose.pose = msg.pose.pose
        self._latest_pose = pose

    def _poll_keyboard(self) -> None:
        readable, _, _ = select.select([self._input_stream], [], [], 0.0)
        if not readable:
            return
        key = self._input_stream.read(1).lower()
        if key == 'c':
            self._capture_current_pose()
        elif key == 'q':
            self._shutdown()

    def _capture_current_pose(self) -> None:
        if self._latest_pose is None:
            self.get_logger().warn('No odometry pose available yet; wait a moment and try again.')
            return

        try:
            transform = self._tf_buffer.lookup_transform(
                self.target_frame,
                self._latest_pose.header.frame_id,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.2),
            )
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f'Could not transform to {self.target_frame}: {exc}')
            return

        transformed_pose = do_transform_pose(self._latest_pose, transform)
        q = transformed_pose.pose.orientation
        yaw = yaw_from_quaternion(q.x, q.y, q.z, q.w)
        name = f'{self.prefix}{self._next_index}'
        self._next_index += 1

        try:
            store = SemanticStore(self.semantic_file)
            store.add_pose(name, transformed_pose.pose.position.x, transformed_pose.pose.position.y, yaw, category='waypoint', description='captured_from_keyboard_path')
        except SemanticStoreError as exc:
            self.get_logger().error(f'Failed to save {name}: {exc}')
            return

        self.get_logger().info(
            f'Captured {name}: x={transformed_pose.pose.position.x:.3f}, '
            f'y={transformed_pose.pose.position.y:.3f}, yaw={yaw:.3f}'
        )

    def _shutdown(self) -> None:
        self.restore_terminal()
        raise KeyboardInterrupt

    def restore_terminal(self) -> None:
        if self._terminal_settings is not None:
            termios.tcsetattr(self._input_stream, termios.TCSADRAIN, self._terminal_settings)
            self._terminal_settings = None
        if self._owns_input_stream:
            self._input_stream.close()
            self._owns_input_stream = False

    def destroy_node(self) -> None:
        self.restore_terminal()
        super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = PathWaypointRecorder()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as exc:  # noqa: BLE001
        print(f'path_waypoint_recorder: {exc}', file=sys.stderr)
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
