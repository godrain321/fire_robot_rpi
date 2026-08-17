"""Record RViz waypoint clicks, display the queue, then execute it sequentially."""

from pathlib import Path as FilePath

from geometry_msgs.msg import PoseArray, PoseStamped
from nav_msgs.msg import Path
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
import yaml

from .waypoint_file import WaypointFileError, validated_pose_values


class WaypointQueue(Node):
    def __init__(self):
        super().__init__('waypoint_queue')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('load_file', '')
        self.map_frame = str(self.get_parameter('map_frame').value)
        self.load_file = str(self.get_parameter('load_file').value)
        qos = QoSProfile(depth=1)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.queue = []
        self.current_index = None
        self.waiting_for_departure = False
        self.edit_first_on_next_click = False
        self.queue_path = self.create_publisher(Path, '/waypoint_queue', qos)
        self.queue_poses = self.create_publisher(PoseArray, '/waypoint_poses', qos)
        self.goal = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.status = self.create_publisher(String, '/waypoint_queue_status', 10)
        self.create_subscription(PoseStamped, '/waypoint_click', self._click, 10)
        self.create_subscription(String, '/waypoint_queue_command', self._command, 10)
        self.create_subscription(String, '/follower_state', self._follower, 10)
        self._load_saved_queue(self.load_file)
        self._publish_queue()
        self._state(
            f'RESTORED:{len(self.queue)}' if self.queue
            else 'EMPTY: press 2, then click RViz 2D Goal Pose'
        )

    def _load_saved_queue(self, filename):
        if not filename:
            return
        try:
            with open(filename, encoding='utf-8') as stream:
                # `ros2 topic echo --once` terminates output with `---`, which
                # makes the saved file a multi-document YAML stream. Use the
                # first non-empty document so those snapshots restore directly.
                document = next(
                    (item for item in yaml.safe_load_all(stream) if item), {}
                )
            values = validated_pose_values(document, self.map_frame)
            restored = []
            for x, y, z, qx, qy, qz, qw in values:
                pose = PoseStamped()
                pose.header.frame_id = self.map_frame
                pose.pose.position.x = x
                pose.pose.position.y = y
                pose.pose.position.z = z
                pose.pose.orientation.x = qx
                pose.pose.orientation.y = qy
                pose.pose.orientation.z = qz
                pose.pose.orientation.w = qw
                restored.append(pose)
            self.queue.extend(restored)
            self.get_logger().info(f'Restored {len(self.queue)} waypoints from {filename}')
        except (OSError, TypeError, ValueError, WaypointFileError, yaml.YAMLError) as exc:
            self.get_logger().error(f'Cannot restore waypoint file {filename}: {exc}')

    def _click(self, message):
        if message.header.frame_id and message.header.frame_id != self.map_frame:
            self._state(f'REJECTED_FRAME:{message.header.frame_id}')
            return
        message.header.frame_id = self.map_frame
        if self.edit_first_on_next_click:
            self.edit_first_on_next_click = False
            if not self.queue:
                self._state('EMPTY:CANNOT_EDIT_FIRST')
                return
            self.queue[0] = message
            self.current_index = None
            self.waiting_for_departure = False
            self._publish_queue()
            if self.load_file:
                try:
                    self._save_queue(self.load_file)
                except (OSError, yaml.YAMLError) as exc:
                    self._state(f'EDITED_FIRST:SAVE_FAILED:{exc}')
                    self.get_logger().error(
                        f'Waypoint 1 changed in memory but YAML save failed: {exc}'
                    )
                    return
            self._state('EDITED_FIRST:SAVED' if self.load_file else 'EDITED_FIRST')
            self.get_logger().info(
                'Waypoint 1 replaced: '
                f'({message.pose.position.x:.3f}, {message.pose.position.y:.3f})'
            )
            return
        self.queue.append(message)
        self.current_index = None
        self._publish_queue()
        self._state(f'RECORDED:{len(self.queue)}')
        self.get_logger().info(
            f'Waypoint {len(self.queue)} recorded: '
            f'({message.pose.position.x:.3f}, {message.pose.position.y:.3f})'
        )

    def _command(self, message):
        command = message.data.strip().upper()
        if command == 'CLEAR':
            self.queue.clear()
            self.current_index = None
            self.waiting_for_departure = False
            self._publish_queue()
            self._state('CLEARED')
        elif command == 'EDIT_FIRST':
            if not self.queue:
                self._state('EMPTY:CANNOT_EDIT_FIRST')
                return
            if self.current_index is not None:
                self._state('RUNNING:CANNOT_EDIT_FIRST')
                return
            self.edit_first_on_next_click = True
            self._state('EDIT_FIRST_READY: click RViz 2D Goal Pose')
        elif command == 'GO':
            if not self.queue:
                self._state('EMPTY:CANNOT_GO')
                return
            self.current_index = 0
            self._send_current_goal()

    def _save_queue(self, filename):
        """Atomically persist the current queue in the loadable PoseArray form."""
        document = {
            'header': {
                'stamp': {'sec': 0, 'nanosec': 0},
                'frame_id': self.map_frame,
            },
            'poses': [],
        }
        for waypoint in self.queue:
            position = waypoint.pose.position
            orientation = waypoint.pose.orientation
            document['poses'].append({
                'header': {
                    'stamp': {'sec': 0, 'nanosec': 0},
                    'frame_id': self.map_frame,
                },
                'pose': {
                    'position': {
                        'x': float(position.x), 'y': float(position.y),
                        'z': float(position.z),
                    },
                    'orientation': {
                        'x': float(orientation.x), 'y': float(orientation.y),
                        'z': float(orientation.z), 'w': float(orientation.w),
                    },
                },
            })
        destination = FilePath(filename)
        temporary = destination.with_suffix(destination.suffix + '.tmp')
        temporary.write_text(
            yaml.safe_dump(document, sort_keys=False), encoding='utf-8'
        )
        temporary.replace(destination)

    def _send_current_goal(self):
        goal = self.queue[self.current_index]
        goal.header.stamp = self.get_clock().now().to_msg()
        self.goal.publish(goal)
        self.waiting_for_departure = True
        self._state(f'RUNNING:{self.current_index + 1}/{len(self.queue)}')

    def _follower(self, message):
        if self.current_index is None:
            return
        # Ignore a retained GOAL_REACHED from the preceding waypoint until the
        # follower has actually accepted/departed toward the newly published one.
        if message.data not in ('GOAL_REACHED', 'WAITING_FOR_PATH'):
            self.waiting_for_departure = False
        if message.data != 'GOAL_REACHED' or self.waiting_for_departure:
            return
        self.current_index += 1
        if self.current_index >= len(self.queue):
            self.current_index = None
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

    def _state(self, state):
        self.status.publish(String(data=state))


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
