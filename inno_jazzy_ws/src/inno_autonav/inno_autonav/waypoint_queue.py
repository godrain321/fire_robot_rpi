"""Record RViz waypoints, persist the queue, and execute it sequentially."""

import os
from pathlib import Path as FilePath
import tempfile

from geometry_msgs.msg import PoseArray, PoseStamped
from nav_msgs.msg import Path
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
import yaml

from .grid_utils import quaternion_from_yaw
from .waypoint_selection import (
    resolve_mode4_waypoints,
    waypoint_names_from_document,
)


def poses_from_document(document, map_frame):
    """Read ros2 PoseArray snapshots or named semantic waypoint YAML."""
    if 'poses' in document:
        raw_poses = document['poses']
        if isinstance(raw_poses, dict):
            entries = raw_poses.values()
            semantic_format = True
        elif isinstance(raw_poses, list):
            entries = raw_poses
            semantic_format = False
        else:
            raise ValueError('poses 형식은 mapping 또는 list여야 합니다.')
    else:
        raw_semantic = document.get('semantic_points', {})
        if not isinstance(raw_semantic, dict):
            raise ValueError('poses 또는 semantic_points 형식이 올바르지 않습니다.')
        entries = raw_semantic.values()
        semantic_format = True

    poses = []
    for entry in entries:
        if not isinstance(entry, dict):
            raise ValueError('waypoint 항목은 mapping이어야 합니다.')
        pose = PoseStamped()
        pose.header.frame_id = str(
            entry.get('frame_id', document.get('frame_id', map_frame))
        ) or map_frame
        if pose.header.frame_id != map_frame:
            raise ValueError(
                f'waypoint frame={pose.header.frame_id!r}; '
                f'{map_frame!r}만 지원합니다.'
            )
        if semantic_format:
            pose.pose.position.x = float(entry['x'])
            pose.pose.position.y = float(entry['y'])
            pose.pose.position.z = float(entry.get('z', 0.0))
            qx, qy, qz, qw = quaternion_from_yaw(float(entry.get('yaw', 0.0)))
            pose.pose.orientation.x = qx
            pose.pose.orientation.y = qy
            pose.pose.orientation.z = qz
            pose.pose.orientation.w = qw
        else:
            # A nav_msgs/Path snapshot wraps each pose in ``pose`` while a
            # geometry_msgs/PoseArray snapshot stores position/orientation
            # directly. Accept both so either ros2 topic echo can be restored.
            source = entry.get('pose', entry)
            position = source.get('position', {})
            orientation = source.get('orientation', {})
            pose.pose.position.x = float(position.get('x', 0.0))
            pose.pose.position.y = float(position.get('y', 0.0))
            pose.pose.position.z = float(position.get('z', 0.0))
            pose.pose.orientation.x = float(orientation.get('x', 0.0))
            pose.pose.orientation.y = float(orientation.get('y', 0.0))
            pose.pose.orientation.z = float(orientation.get('z', 0.0))
            pose.pose.orientation.w = float(orientation.get('w', 1.0))
        poses.append(pose)
    return poses


def document_from_poses(poses, map_frame):
    """Return a stable, human-readable Path-compatible queue snapshot."""
    entries = []
    for stamped_pose in poses:
        pose = stamped_pose.pose
        entries.append(
            {
                'header': {'frame_id': map_frame},
                'pose': {
                    'position': {
                        'x': float(pose.position.x),
                        'y': float(pose.position.y),
                        'z': float(pose.position.z),
                    },
                    'orientation': {
                        'x': float(pose.orientation.x),
                        'y': float(pose.orientation.y),
                        'z': float(pose.orientation.z),
                        'w': float(pose.orientation.w),
                    },
                },
            }
        )
    return {'version': 1, 'frame_id': map_frame, 'poses': entries}


def save_pose_document(filename, poses, map_frame):
    """Atomically persist a waypoint queue without leaving a partial YAML."""
    target = FilePath(filename).expanduser()
    target.parent.mkdir(parents=True, exist_ok=True)
    temporary = None
    try:
        descriptor, temporary = tempfile.mkstemp(
            prefix=f'.{target.name}.', suffix='.tmp', dir=target.parent
        )
        with os.fdopen(descriptor, 'w', encoding='utf-8') as stream:
            yaml.safe_dump(
                document_from_poses(poses, map_frame),
                stream,
                allow_unicode=True,
                sort_keys=False,
            )
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, target)
        temporary = None
    finally:
        if temporary is not None:
            try:
                os.unlink(temporary)
            except FileNotFoundError:
                pass


class WaypointQueue(Node):
    def __init__(self):
        super().__init__('waypoint_queue')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('load_file', '')
        self.declare_parameter('save_file', '')
        self.declare_parameter('preview_goal_index', -1)
        self.declare_parameter('preview_delay_sec', 2.0)
        self.map_frame = str(self.get_parameter('map_frame').value)
        self.save_file = str(self.get_parameter('save_file').value).strip()
        qos = QoSProfile(depth=1)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.queue = []
        self.waypoint_names = []
        self.current_index = None
        self.waiting_for_departure = False
        self.mode4_names = []
        self.mode4_indices = []
        self.mode4_next_position = 0
        self.mode4_current_position = None
        self.queue_path = self.create_publisher(Path, '/waypoint_queue', qos)
        self.queue_poses = self.create_publisher(
            PoseArray, '/waypoint_poses', qos
        )
        self.goal = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.step_index = 0
        self.execution_mode = 'continuous'
        self.status = self.create_publisher(
            String, '/waypoint_queue_status', 10
        )
        self.create_subscription(
            PoseStamped, '/waypoint_click', self._click, 10
        )
        self.create_subscription(
            String, '/waypoint_queue_command', self._command, 10
        )
        self.create_subscription(
            String, '/follower_state', self._follower, 10
        )
        self.markers = self.create_publisher(
            MarkerArray, '/waypoint_markers', qos
        )
        self._load_saved_queue(str(self.get_parameter('load_file').value))
        self._publish_queue()
        self._state(
            f'RESTORED:{len(self.queue)}' if self.queue
            else 'EMPTY: press 2, then click RViz 2D Goal Pose'
        )
        preview_index = int(self.get_parameter('preview_goal_index').value)
        preview_delay = float(self.get_parameter('preview_delay_sec').value)
        if preview_index >= 0:
            if preview_index >= len(self.queue):
                self.get_logger().error(
                    f'preview_goal_index={preview_index}, '
                    f'queue size={len(self.queue)}'
                )
            elif preview_delay <= 0.0:
                self.get_logger().error('preview_delay_sec는 0보다 커야 합니다.')
            else:
                self.preview_goal_index = preview_index
                self.preview_timer = self.create_timer(
                    preview_delay, self._publish_preview_goal
                )

    def _publish_preview_goal(self):
        self.preview_timer.cancel()
        goal = self.queue[self.preview_goal_index]
        goal.header.stamp = self.get_clock().now().to_msg()
        self.goal.publish(goal)
        self._state(
            f'PREVIEW_GOAL:{self.preview_goal_index + 1}/{len(self.queue)}'
        )
        self.get_logger().info(
            f'Published preview goal #{self.preview_goal_index + 1}'
        )

    def _load_saved_queue(self, filename):
        if not filename:
            return
        expanded = FilePath(filename).expanduser()
        if not expanded.exists():
            if (
                self.save_file
                and expanded == FilePath(self.save_file).expanduser()
            ):
                self.get_logger().info(
                    'Waypoint save file will be created on first click: '
                    f'{expanded}'
                )
            else:
                self.get_logger().error(
                    f'Waypoint file does not exist: {expanded}'
                )
            return
        try:
            with expanded.open(encoding='utf-8') as stream:
                # `ros2 topic echo --once` terminates output with `---`, which
                # makes the saved file a multi-document YAML stream. Use the
                # first non-empty document so snapshots restore directly.
                document = next(
                    (item for item in yaml.safe_load_all(stream) if item), {}
                )
            loaded_poses = poses_from_document(document, self.map_frame)
            loaded_names = waypoint_names_from_document(document)
            if len(loaded_names) != len(loaded_poses):
                raise ValueError('waypoint 이름과 pose 개수가 다릅니다.')
            self.queue.extend(loaded_poses)
            self.waypoint_names.extend(loaded_names)
            self.get_logger().info(
                f'Restored {len(self.queue)} waypoints from {expanded}'
            )
        except (OSError, TypeError, ValueError, yaml.YAMLError) as exc:
            self.get_logger().error(
                f'Cannot restore waypoint file {expanded}: {exc}'
            )

    def _save_queue(self):
        if not self.save_file:
            return
        try:
            save_pose_document(self.save_file, self.queue, self.map_frame)
        except (OSError, TypeError, ValueError, yaml.YAMLError) as exc:
            self._state(f'SAVE_FAILED:{exc}')
            self.get_logger().error(
                f'Cannot save waypoint queue to {self.save_file}: {exc}'
            )
            return
        self.get_logger().info(
            f'Saved {len(self.queue)} waypoints to '
            f'{FilePath(self.save_file).expanduser()}'
        )

    def _click(self, message):
        if self.execution_mode == 'mode4' and self.current_index is not None:
            self._state('MODE4_REJECTED:BUSY:CANNOT_RECORD')
            return
        if (
            message.header.frame_id
            and message.header.frame_id != self.map_frame
        ):
            self._state(f'REJECTED_FRAME:{message.header.frame_id}')
            return
        message.header.frame_id = self.map_frame
        self.queue.append(message)
        used_names = {name.casefold() for name in self.waypoint_names}
        next_number = len(self.queue)
        while f'w{next_number}'.casefold() in used_names:
            next_number += 1
        self.waypoint_names.append(f'w{next_number}')
        self.current_index = None
        self._reset_mode4_selection()
        self._save_queue()
        self._publish_queue()
        self._state(f'RECORDED:{len(self.queue)}')
        self.get_logger().info(
            f'Waypoint {len(self.queue)} recorded: '
            f'({message.pose.position.x:.3f}, {message.pose.position.y:.3f})'
        )

    def _command(self, message):
        raw_command = message.data.strip()
        command = raw_command.upper()
        if command.startswith('MODE4_SET:'):
            self._start_mode4(raw_command.split(':', 1)[1])
            return
        if command == 'MODE4_NEXT':
            self._mode4_next()
            return
        if command == 'MODE4_CANCEL':
            if self.current_index is not None:
                self._state('MODE4_REJECTED:BUSY:PRESS_1_TO_STOP')
                return
            self._reset_mode4_selection()
            self.execution_mode = 'continuous'
            self._publish_queue()
            self._state('MODE4_CANCELLED')
            return
        if command == 'CLEAR':
            self.queue.clear()
            self.waypoint_names.clear()
            self.current_index = None
            self.waiting_for_departure = False
            self.step_index = 0
            self.execution_mode = 'continuous'
            self._reset_mode4_selection()
            self._save_queue()
            self._publish_queue()
            self._state('CLEARED')
        elif command == 'GO':
            if not self.queue:
                self._state('EMPTY:CANNOT_GO')
                return
            self._reset_mode4_selection()
            self.step_index = 0
            self.execution_mode = 'continuous'
            self.current_index = 0
            self._send_current_goal()
        elif command == 'STEP':
            if not self.queue:
                self._state('EMPTY:CANNOT_STEP')
                return
            if self.current_index is not None:
                self._state(f'BUSY:{self.current_index + 1}/{len(self.queue)}')
                return
            if self.step_index >= len(self.queue):
                self.step_index = 0
            self._reset_mode4_selection()
            self.execution_mode = 'step'
            self.current_index = self.step_index
            self._send_current_goal()

    def _reset_mode4_selection(self):
        self.mode4_names = []
        self.mode4_indices = []
        self.mode4_next_position = 0
        self.mode4_current_position = None

    def _start_mode4(self, selection_text):
        if self.current_index is not None:
            self._state('MODE4_REJECTED:BUSY')
            return
        try:
            names, indices = resolve_mode4_waypoints(
                selection_text, self.waypoint_names
            )
        except ValueError as error:
            self._state(f'MODE4_REJECTED:{error}')
            return
        self.mode4_names = names
        self.mode4_indices = indices
        self.mode4_next_position = 0
        self.mode4_current_position = None
        self.step_index = 0
        self.execution_mode = 'mode4'
        self._publish_queue()
        self._state('MODE4_ACCEPTED:' + '->'.join(self.mode4_names))
        self._send_next_mode4_goal()

    def _mode4_next(self):
        if self.execution_mode != 'mode4' or not self.mode4_indices:
            self._state('MODE4_REJECTED:NO_SELECTION:PRESS_4')
            return
        if self.current_index is not None:
            active = self.mode4_names[self.mode4_current_position]
            self._state(f'MODE4_BUSY:{active}')
            return
        if self.mode4_next_position >= len(self.mode4_indices):
            self._state('MODE4_MISSION_COMPLETE')
            return
        self._send_next_mode4_goal()

    def _send_next_mode4_goal(self):
        position = self.mode4_next_position
        self.mode4_current_position = position
        self.current_index = self.mode4_indices[position]
        self._send_current_goal()

    def _send_current_goal(self):
        goal = self.queue[self.current_index]
        goal.header.stamp = self.get_clock().now().to_msg()
        self.goal.publish(goal)
        self.waiting_for_departure = True
        self._publish_queue()
        if self.execution_mode == 'mode4':
            position = self.mode4_current_position
            name = self.mode4_names[position]
            self._state(
                f'MODE4_RUNNING:{position + 1}/{len(self.mode4_names)}:{name}'
            )
        else:
            self._state(f'RUNNING:{self.current_index + 1}/{len(self.queue)}')

    def _follower(self, message):
        if self.current_index is None:
            return
        if message.data in (
            'PATH_ACCEPTED', 'FOLLOWING_PATH', 'ROTATING_IN_PLACE',
            'ALIGNING_GOAL_YAW',
        ):
            self.waiting_for_departure = False
        if message.data != 'GOAL_REACHED' or self.waiting_for_departure:
            return
        if self.execution_mode == 'mode4':
            completed_position = self.mode4_current_position
            completed_name = self.mode4_names[completed_position]
            self.mode4_next_position = completed_position + 1
            self.mode4_current_position = None
            self.current_index = None
            self._publish_queue()
            if self.mode4_next_position >= len(self.mode4_names):
                self._state(
                    f'MODE4_MISSION_COMPLETE:{completed_name}'
                )
            else:
                next_name = self.mode4_names[self.mode4_next_position]
                self._state(
                    f'MODE4_REACHED:{completed_name}:SPACE_FOR:{next_name}'
                )
            return
        completed = self.current_index
        if self.execution_mode == 'step':
            self.step_index = completed + 1
            self.current_index = None
            self._publish_queue()
            if self.step_index >= len(self.queue):
                self._state('STEP_MISSION_COMPLETE')
            else:
                self._state(
                    f'STEP_COMPLETE:{completed + 1}/{len(self.queue)}:'
                    f'SPACE_FOR:{self.step_index + 1}'
                )
            return
        self.current_index += 1
        if self.current_index >= len(self.queue):
            self.current_index = None
            self.step_index = len(self.queue)
            self._publish_queue()
            self._state('MISSION_COMPLETE')
            return
        self._send_current_goal()

    def _publish_queue(self):
        path = Path()
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = self.map_frame
        path.poses = list(self.queue)
        self.queue_path.publish(path)
        poses = PoseArray()
        poses.header = path.header
        poses.poses = [waypoint.pose for waypoint in self.queue]
        self.queue_poses.publish(poses)
        marker_array = MarkerArray()
        clear = Marker()
        clear.header = path.header
        clear.action = Marker.DELETEALL
        marker_array.markers.append(clear)
        completed_mode4 = set(
            self.mode4_indices[:self.mode4_next_position]
        )
        pending_mode4 = set(
            self.mode4_indices[self.mode4_next_position:]
        )
        for index, waypoint in enumerate(self.queue):
            marker = Marker()
            marker.header = path.header
            marker.ns = 'waypoint_numbers'
            marker.id = index
            marker.type = Marker.TEXT_VIEW_FACING
            marker.action = Marker.ADD
            marker.pose.position.x = waypoint.pose.position.x
            marker.pose.position.y = waypoint.pose.position.y
            marker.pose.position.z = 0.45
            marker.pose.orientation.w = 1.0
            marker.scale.z = 0.35
            name = (
                self.waypoint_names[index]
                if index < len(self.waypoint_names)
                else f'w{index + 1}'
            )
            if index == self.current_index:
                marker.text = f'ACTIVE {name}'
                marker.color.r, marker.color.g, marker.color.b = 0.1, 1.0, 0.1
            elif self.execution_mode == 'mode4' and index in pending_mode4:
                marker.text = f'SELECTED {name}'
                marker.color.r, marker.color.g, marker.color.b = 0.1, 0.7, 1.0
            elif self.execution_mode == 'mode4' and index in completed_mode4:
                marker.text = f'DONE {name}'
                marker.color.r = marker.color.g = marker.color.b = 0.5
            elif index < self.step_index:
                marker.text = f'DONE {name}'
                marker.color.r = marker.color.g = marker.color.b = 0.5
            else:
                marker.text = name
                marker.color.r, marker.color.g, marker.color.b = 1.0, 0.85, 0.0
            marker.color.a = 1.0
            marker_array.markers.append(marker)
        self.markers.publish(marker_array)

    def _state(self, state):
        self.status.publish(String(data=state))
        self.get_logger().info(state)


def main(args=None):
    rclpy.init(args=args)
    node = WaypointQueue()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
