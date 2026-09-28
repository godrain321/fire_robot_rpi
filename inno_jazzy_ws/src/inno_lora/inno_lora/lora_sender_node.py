from collections import OrderedDict
import json
import math
import re

import numpy as np
import serial

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from inno_autonav.evacuation_demo import exit_visualization_records
from inno_autonav.exit_evaluator import ExitEvaluationBatch
from inno_hazard.hazard_snapshot import decode_hazard_snapshot_message
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float32MultiArray, String


ACTIVE_THERMAL_STATUSES = {'ACTIVE', 'ACTIVE_THERMAL_ONLY'}
HUMAN_STATES = {'CONFIRMED', 'ASSIST_CHECK'}
EXIT_STATES = {'UNKNOWN', 'USABLE', 'BLOCKED', 'DANGEROUS', 'DANGER_EXPECTED'}
TOKEN_PATTERN = re.compile(r'^[A-Za-z0-9_-]+$')


def thermal_state(temperature, safe_temperature, blocked_temperature):
    """Map the existing ROS thermal bands to compact LoRa state codes."""
    values = (temperature, safe_temperature, blocked_temperature)
    if not all(math.isfinite(float(value)) for value in values):
        raise ValueError('thermal thresholds and temperature must be finite')
    if blocked_temperature <= safe_temperature:
        raise ValueError('blocked temperature must exceed safe temperature')
    if temperature <= safe_temperature:
        return 0
    if temperature >= blocked_temperature:
        return 2
    return 1


def representative_thermal_from_snapshot(message):
    """Return the hottest observed map cell, or None when thermal is invalid."""
    metadata, layers = decode_hazard_snapshot_message(message)
    if metadata['status'] not in ACTIVE_THERMAL_STATUSES:
        return None
    if str(metadata['frame_id']).strip().lstrip('/') != 'map':
        raise ValueError('thermal snapshot frame must be map')

    temperatures = np.asarray(layers['temperature_c'], dtype=float)
    observed = np.asarray(layers['temperature_observed'], dtype=float) >= 0.5
    candidates = observed & np.isfinite(temperatures)
    if not np.any(candidates):
        return None

    ranked = np.where(candidates, temperatures, -np.inf)
    row, column = np.unravel_index(int(np.argmax(ranked)), ranked.shape)
    temperature = float(temperatures[row, column])
    resolution = float(metadata['resolution'])
    origin_x = float(metadata['origin_x'])
    origin_y = float(metadata['origin_y'])
    origin_yaw = float(metadata['origin_yaw'])
    if not all(math.isfinite(value) for value in (
        resolution, origin_x, origin_y, origin_yaw,
    )) or resolution <= 0.0:
        raise ValueError('thermal snapshot geometry is invalid')

    local_x = (column + 0.5) * resolution
    local_y = (row + 0.5) * resolution
    cosine = math.cos(origin_yaw)
    sine = math.sin(origin_yaw)
    world_x = origin_x + cosine * local_x - sine * local_y
    world_y = origin_y + sine * local_x + cosine * local_y
    state = thermal_state(
        temperature,
        float(metadata['temperature_safe_c']),
        float(metadata['temperature_blocked_c']),
    )
    return world_x, world_y, temperature, state


def _valid_token(value, label):
    token = str(value).strip()
    if not TOKEN_PATTERN.fullmatch(token):
        raise ValueError(f'{label} contains unsupported characters')
    return token


def parse_human_snapshot(payload, expected_frame='map'):
    try:
        document = json.loads(str(payload))
        frame = str(document['frame_id'])
        raw_humans = document['humans']
    except (KeyError, TypeError, ValueError, json.JSONDecodeError) as exc:
        raise ValueError('invalid human snapshot') from exc
    if frame.lstrip('/') != str(expected_frame).lstrip('/'):
        raise ValueError('human snapshot frame must be map')
    if not isinstance(raw_humans, list):
        raise ValueError('human snapshot list is invalid')
    humans = {}
    for raw in raw_humans:
        try:
            identifier = _valid_token(raw['id'], 'human id')
            x = float(raw['x'])
            y = float(raw['y'])
            state = str(raw['state']).upper()
        except (KeyError, TypeError, ValueError) as exc:
            raise ValueError('invalid human record') from exc
        if identifier in humans or state not in HUMAN_STATES:
            raise ValueError('duplicate human id or unknown human state')
        if not math.isfinite(x) or not math.isfinite(y):
            raise ValueError('human coordinates must be finite')
        humans[identifier] = (x, y, state)
    return humans


def parse_exit_snapshot(payload, expected_frame='map'):
    try:
        document = json.loads(str(payload))
        batch = ExitEvaluationBatch.from_dict(document, expected_frame)
    except (TypeError, ValueError, json.JSONDecodeError) as exc:
        raise ValueError('invalid exit evaluation snapshot') from exc
    records = exit_visualization_records(payload, expected_frame)
    if len(records) != len(batch.evaluations):
        raise ValueError('exit visualization records are incomplete')
    exits = {}
    for raw_identifier, position, state in records:
        identifier = _valid_token(raw_identifier, 'exit id')
        if identifier in exits or state not in EXIT_STATES:
            raise ValueError('duplicate exit id or unknown exit state')
        x, y = position
        exits[identifier] = (float(x), float(y), state)
    return exits


def _format_records(records):
    return ';'.join(
        f'{identifier}:{x:.3f}:{y:.3f}:{state}'
        for identifier, (x, y, state) in sorted(records.items())
    )


def format_status_packet(
    sequence, pose, thermal, human_valid, humans, exit_valid, exits,
):
    x, y, yaw = pose
    prefix = f'STATUS,{sequence},{x:.3f},{y:.3f},{yaw:.3f}'
    if thermal is None:
        thermal_fields = '0,,,,'
    else:
        tx, ty, temperature, state = thermal
        thermal_fields = f'1,{tx:.3f},{ty:.3f},{temperature:.1f},{state}'
    human_fields = (
        f'{int(human_valid)},{len(humans) if human_valid else 0},'
        f'{_format_records(humans) if human_valid else ""}'
    )
    exit_fields = (
        f'{int(exit_valid)},{len(exits) if exit_valid else 0},'
        f'{_format_records(exits) if exit_valid else ""}'
    )
    return f'{prefix},{thermal_fields},{human_fields},{exit_fields}'


class LoraSender(Node):
    def __init__(self) -> None:
        super().__init__('lora_sender')
        defaults = {
            'serial_port': '/dev/ttyUSB0',
            'baud_rate': 9600,
            'send_period_sec': 2.0,
            'pose_topic': '/amcl_pose',
            'thermal_snapshot_topic': '/hazard/snapshot',
            'human_snapshot_topic': '/human/tracks',
            'exit_evaluations_topic': '/exit_evaluations',
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.serial_port = self.get_parameter('serial_port').value
        self.baud_rate = self.get_parameter('baud_rate').value
        self.send_period_sec = self.get_parameter('send_period_sec').value
        self.pose_topic = self.get_parameter('pose_topic').value
        self.thermal_snapshot_topic = self.get_parameter(
            'thermal_snapshot_topic'
        ).value
        self.human_snapshot_topic = self.get_parameter(
            'human_snapshot_topic'
        ).value
        self.exit_evaluations_topic = self.get_parameter(
            'exit_evaluations_topic'
        ).value

        self.serial_connection = None
        self.status_sequence = 1
        self.temp_sequence = 1
        self.human_sequence = 1
        self.exit_sequence = 1
        self.latest_x = self.latest_y = self.latest_yaw = None
        self.have_pose = False
        self.waiting_for_pose_logged = False
        self.invalid_pose_logged = False
        self.latest_thermal_valid = False
        self.latest_thermal_x = self.latest_thermal_y = None
        self.latest_temperature = self.latest_thermal_state = None
        self.previous_thermal_state = None
        self.pending_thermal_event = False
        self.invalid_thermal_logged = False
        self.latest_humans = {}
        self.have_human_snapshot = False
        self.invalid_human_logged = False
        self.latest_exits = {}
        self.have_exit_snapshot = False
        self.invalid_exit_logged = False
        self.pending_events = OrderedDict()

        self.get_logger().info('[inno_lora] LoRa sender started')
        self.get_logger().info(f'Serial port: {self.serial_port}')
        self.get_logger().info(f'Baud rate: {self.baud_rate}')
        self.get_logger().info(f'Send period: {self.send_period_sec} sec')
        self.get_logger().info(f'Pose topic: {self.pose_topic}')
        self.get_logger().info(
            f'Thermal snapshot topic: {self.thermal_snapshot_topic}'
        )
        self.get_logger().info(f'Human snapshot topic: {self.human_snapshot_topic}')
        self.get_logger().info(
            f'Exit evaluations topic: {self.exit_evaluations_topic}'
        )

        snapshot_qos = QoSProfile(depth=1)
        snapshot_qos.reliability = ReliabilityPolicy.RELIABLE
        snapshot_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.pose_subscription = self.create_subscription(
            PoseWithCovarianceStamped, self.pose_topic, self._pose_callback, 10
        )
        self.thermal_subscription = self.create_subscription(
            Float32MultiArray, self.thermal_snapshot_topic,
            self._thermal_callback, snapshot_qos,
        )
        self.human_subscription = self.create_subscription(
            String, self.human_snapshot_topic, self._human_callback, snapshot_qos
        )
        self.exit_subscription = self.create_subscription(
            String, self.exit_evaluations_topic, self._exit_callback, snapshot_qos
        )
        self._open_serial()
        self.status_timer = self.create_timer(
            self.send_period_sec, self._send_status
        )
        self.event_timer = self.create_timer(0.05, self._send_pending_event)

    def _pose_callback(self, message: PoseWithCovarianceStamped) -> None:
        position = message.pose.pose.position
        orientation = message.pose.pose.orientation
        values = (
            position.x, position.y, orientation.x, orientation.y,
            orientation.z, orientation.w,
        )
        norm_squared = sum(value * value for value in values[2:])
        if not all(math.isfinite(value) for value in values) or norm_squared <= 1e-12:
            if not self.invalid_pose_logged:
                self.get_logger().warning(
                    'Ignoring localization pose with invalid position or quaternion'
                )
                self.invalid_pose_logged = True
            return
        inverse_norm = 1.0 / math.sqrt(norm_squared)
        qx, qy, qz, qw = (
            value * inverse_norm for value in values[2:]
        )
        self.latest_x = float(position.x)
        self.latest_y = float(position.y)
        self.latest_yaw = math.atan2(
            2.0 * (qw * qz + qx * qy),
            1.0 - 2.0 * (qy * qy + qz * qz),
        )
        self.have_pose = True
        self.invalid_pose_logged = False

    def _thermal_callback(self, message: Float32MultiArray) -> None:
        try:
            sample = representative_thermal_from_snapshot(message)
        except (KeyError, TypeError, ValueError, IndexError) as exc:
            self.latest_thermal_valid = False
            self.pending_thermal_event = False
            if not self.invalid_thermal_logged:
                self.get_logger().warning(f'Ignoring invalid thermal snapshot: {exc}')
                self.invalid_thermal_logged = True
            return
        self.invalid_thermal_logged = False
        if sample is None:
            self.latest_thermal_valid = False
            self.pending_thermal_event = False
            return
        x, y, temperature, state = sample
        state_changed = (
            self.previous_thermal_state is not None
            and state != self.previous_thermal_state
        )
        self.latest_thermal_x, self.latest_thermal_y = x, y
        self.latest_temperature, self.latest_thermal_state = temperature, state
        self.latest_thermal_valid = True
        self.previous_thermal_state = state
        if state_changed:
            self.pending_thermal_event = True

    def _human_callback(self, message: String) -> None:
        try:
            humans = parse_human_snapshot(message.data)
        except ValueError as exc:
            if not self.invalid_human_logged:
                self.get_logger().warning(f'Ignoring invalid human snapshot: {exc}')
                self.invalid_human_logged = True
            return
        previous = self.latest_humans
        for identifier, record in humans.items():
            self.pending_events.pop(('HUMAN_REMOVE', identifier), None)
            if identifier not in previous or record[2] != previous[identifier][2]:
                self.pending_events[('HUMAN', identifier)] = ('HUMAN', identifier, record)
            elif ('HUMAN', identifier) in self.pending_events:
                self.pending_events[('HUMAN', identifier)] = ('HUMAN', identifier, record)
        for identifier in set(previous) - set(humans):
            self.pending_events.pop(('HUMAN', identifier), None)
            self.pending_events[('HUMAN_REMOVE', identifier)] = (
                'HUMAN_REMOVE', identifier, None
            )
        self.latest_humans = humans
        self.have_human_snapshot = True
        self.invalid_human_logged = False

    def _exit_callback(self, message: String) -> None:
        try:
            exits = parse_exit_snapshot(message.data)
        except ValueError as exc:
            if not self.invalid_exit_logged:
                self.get_logger().warning(f'Ignoring invalid exit snapshot: {exc}')
                self.invalid_exit_logged = True
            return
        if self.have_exit_snapshot:
            for identifier, record in exits.items():
                old = self.latest_exits.get(identifier)
                if old is not None and record[2] != old[2]:
                    self.pending_events[('EXIT', identifier)] = (
                        'EXIT', identifier, record
                    )
                elif ('EXIT', identifier) in self.pending_events:
                    self.pending_events[('EXIT', identifier)] = (
                        'EXIT', identifier, record
                    )
        self.latest_exits = exits
        self.have_exit_snapshot = True
        self.invalid_exit_logged = False

    def _open_serial(self) -> bool:
        try:
            self.serial_connection = serial.Serial(
                port=self.serial_port, baudrate=self.baud_rate,
                bytesize=serial.EIGHTBITS, parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE, timeout=1.0, write_timeout=1.0,
            )
        except (serial.SerialException, OSError) as exc:
            self.serial_connection = None
            self.get_logger().error(
                f'Failed to open serial port {self.serial_port}: {exc}. '
                f'Retrying in {self.send_period_sec} sec.'
            )
            return False
        self.get_logger().info(f'Opened serial port {self.serial_port}')
        return True

    def _write_packet(self, message: str) -> bool:
        if self.serial_connection is None or not self.serial_connection.is_open:
            return False
        payload = f'{message}\n'.encode('ascii')
        try:
            bytes_written = self.serial_connection.write(payload)
            self.serial_connection.flush()
            if bytes_written != len(payload):
                raise serial.SerialTimeoutException(
                    f'wrote {bytes_written} of {len(payload)} bytes'
                )
        except (serial.SerialException, OSError) as exc:
            self.get_logger().error(f'Serial write failed: {exc}')
            self.close_serial()
            return False
        self.get_logger().info(f'TX: {message}')
        return True

    def _send_status(self) -> None:
        if not self.have_pose:
            if not self.waiting_for_pose_logged:
                self.get_logger().warning('Waiting for localization pose...')
                self.waiting_for_pose_logged = True
            return
        if self.serial_connection is None or not self.serial_connection.is_open:
            if not self._open_serial():
                return
        thermal = None
        if self.latest_thermal_valid:
            thermal = (
                self.latest_thermal_x, self.latest_thermal_y,
                self.latest_temperature, self.latest_thermal_state,
            )
        message = format_status_packet(
            self.status_sequence,
            (self.latest_x, self.latest_y, self.latest_yaw),
            thermal,
            self.have_human_snapshot,
            self.latest_humans,
            self.have_exit_snapshot,
            self.latest_exits,
        )
        if self._write_packet(message):
            self.status_sequence += 1

    def _send_pending_event(self) -> None:
        if self.pending_thermal_event and self.latest_thermal_valid:
            message = (
                f'TEMP,{self.temp_sequence},{self.latest_thermal_x:.3f},'
                f'{self.latest_thermal_y:.3f},{self.latest_temperature:.1f},'
                f'{self.latest_thermal_state}'
            )
            if self._write_packet(message):
                self.temp_sequence += 1
                self.pending_thermal_event = False
            return
        if not self.pending_events:
            return
        key, event = next(iter(self.pending_events.items()))
        kind, identifier, record = event
        if kind == 'HUMAN_REMOVE':
            message = f'HUMAN_REMOVE,{self.human_sequence},{identifier}'
            sequence_name = 'human_sequence'
        elif kind == 'HUMAN':
            x, y, state = record
            message = (
                f'HUMAN,{self.human_sequence},{identifier},'
                f'{x:.3f},{y:.3f},{state}'
            )
            sequence_name = 'human_sequence'
        else:
            x, y, state = record
            message = (
                f'EXIT,{self.exit_sequence},{identifier},'
                f'{x:.3f},{y:.3f},{state}'
            )
            sequence_name = 'exit_sequence'
        if self._write_packet(message):
            setattr(self, sequence_name, getattr(self, sequence_name) + 1)
            self.pending_events.pop(key, None)

    def close_serial(self) -> None:
        if self.serial_connection is None:
            return
        try:
            if self.serial_connection.is_open:
                self.serial_connection.close()
        except (serial.SerialException, OSError) as exc:
            self.get_logger().warning(f'Failed to close serial port: {exc}')
        finally:
            self.serial_connection = None


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LoraSender()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close_serial()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
