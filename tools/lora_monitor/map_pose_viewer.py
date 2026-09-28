#!/usr/bin/env python3
import argparse
from dataclasses import dataclass
from datetime import datetime
import math
from pathlib import Path
import threading
import time

import yaml
from PIL import Image, ImageDraw, ImageOps


THERMAL_NAMES = {0: 'SAFE', 1: 'ELEVATED', 2: 'BLOCKED'}
THERMAL_COLORS = {0: '#2ca02c', 1: '#ff8c00', 2: '#d62728'}
HUMAN_STATES = {'CONFIRMED', 'ASSIST_CHECK'}
HUMAN_COLORS = {'CONFIRMED': '#0066ff', 'ASSIST_CHECK': '#ffd700'}
EXIT_STATES = {'UNKNOWN', 'USABLE', 'BLOCKED', 'DANGEROUS', 'DANGER_EXPECTED'}
EXIT_COLORS = {
    'UNKNOWN': '#808080', 'USABLE': '#22aa22', 'BLOCKED': '#cc2222',
    'DANGEROUS': '#ff8c00', 'DANGER_EXPECTED': '#ff8c00',
}


@dataclass(frozen=True)
class MapData:
    yaml_path: Path
    image_path: Path
    image: Image.Image
    resolution: float
    origin_x: float
    origin_y: float
    origin_yaw: float
    negate: int
    occupied_thresh: float
    free_thresh: float

    @property
    def width(self):
        return self.image.width

    @property
    def height(self):
        return self.image.height


@dataclass(frozen=True)
class PosePacket:
    sequence: int
    x: float
    y: float
    yaw: float
    received_at: float


@dataclass(frozen=True)
class ThermalPacket:
    sequence: int
    x: float
    y: float
    temperature: float
    state: int
    received_at: float


@dataclass(frozen=True)
class HumanPacket:
    sequence: int
    identifier: str
    x: float
    y: float
    state: str
    received_at: float


@dataclass(frozen=True)
class HumanRemovePacket:
    sequence: int
    identifier: str
    received_at: float


@dataclass(frozen=True)
class ExitPacket:
    sequence: int
    identifier: str
    x: float
    y: float
    state: str
    received_at: float


@dataclass(frozen=True)
class StatusPacket:
    sequence: int
    x: float
    y: float
    yaw: float
    thermal: ThermalPacket | None
    human_valid: bool
    humans: tuple[HumanPacket, ...]
    exit_valid: bool
    exits: tuple[ExitPacket, ...]
    received_at: float


def load_map(yaml_path):
    yaml_path = Path(yaml_path).expanduser().resolve()
    with yaml_path.open('r', encoding='utf-8') as stream:
        metadata = yaml.safe_load(stream)
    if not isinstance(metadata, dict):
        raise ValueError('map YAML must contain a mapping')

    required = (
        'image', 'resolution', 'origin', 'negate',
        'occupied_thresh', 'free_thresh',
    )
    missing = [key for key in required if key not in metadata]
    if missing:
        raise ValueError(f'map YAML is missing: {", ".join(missing)}')

    image_path = Path(str(metadata['image'])).expanduser()
    if not image_path.is_absolute():
        image_path = yaml_path.parent / image_path
    image_path = image_path.resolve()

    resolution = float(metadata['resolution'])
    origin = metadata['origin']
    if resolution <= 0.0 or not math.isfinite(resolution):
        raise ValueError('map resolution must be a finite positive number')
    if not isinstance(origin, (list, tuple)) or len(origin) < 3:
        raise ValueError('map origin must be [x, y, yaw]')
    origin_x, origin_y, origin_yaw = (float(value) for value in origin[:3])
    if not all(math.isfinite(value) for value in (origin_x, origin_y, origin_yaw)):
        raise ValueError('map origin values must be finite')
    negate = int(metadata['negate'])
    occupied_thresh = float(metadata['occupied_thresh'])
    free_thresh = float(metadata['free_thresh'])
    if negate not in (0, 1):
        raise ValueError('map negate must be 0 or 1')
    if not 0.0 <= free_thresh < occupied_thresh <= 1.0:
        raise ValueError('map thresholds must satisfy 0 <= free < occupied <= 1')

    with Image.open(image_path) as source:
        image = source.convert('L').copy()
    if negate:
        image = ImageOps.invert(image)

    return MapData(
        yaml_path, image_path, image, resolution,
        origin_x, origin_y, origin_yaw, negate,
        occupied_thresh, free_thresh,
    )


def world_to_pixel(map_data, x_world, y_world):
    dx = x_world - map_data.origin_x
    dy = y_world - map_data.origin_y
    cosine = math.cos(map_data.origin_yaw)
    sine = math.sin(map_data.origin_yaw)
    x_map = cosine * dx + sine * dy
    y_map = -sine * dx + cosine * dy
    return (
        x_map / map_data.resolution,
        map_data.height - 1 - y_map / map_data.resolution,
    )


def pixel_is_in_bounds(map_data, pixel_x, pixel_y):
    return 0.0 <= pixel_x < map_data.width and 0.0 <= pixel_y < map_data.height


def heading_endpoint(map_data, pixel_x, pixel_y, robot_yaw, length_pixels=18.0):
    relative_yaw = robot_yaw - map_data.origin_yaw
    return (
        pixel_x + length_pixels * math.cos(relative_yaw),
        pixel_y - length_pixels * math.sin(relative_yaw),
    )


def _parse_int(value):
    result = int(value)
    if result < 0:
        raise ValueError('sequence must be non-negative')
    return result


def _parse_finite(values):
    result = tuple(float(value) for value in values)
    if not all(math.isfinite(value) for value in result):
        raise ValueError('numeric fields must be finite')
    return result


def _valid_identifier(value):
    return bool(value) and value.isascii() and all(
        character.isalnum() or character in '_-' for character in value
    )


def parse_pose_packet(line, received_at=None):
    fields = line.strip().split(',')
    if len(fields) != 5 or fields[0] != 'POSE':
        return None
    try:
        sequence = _parse_int(fields[1])
        x, y, yaw = _parse_finite(fields[2:])
    except ValueError:
        return None
    return PosePacket(
        sequence, x, y, yaw,
        time.time() if received_at is None else received_at,
    )


def parse_temp_packet(line, received_at=None):
    fields = line.strip().split(',')
    if len(fields) != 6 or fields[0] != 'TEMP':
        return None
    try:
        sequence = _parse_int(fields[1])
        x, y, temperature = _parse_finite(fields[2:5])
        state = int(fields[5])
    except ValueError:
        return None
    if state not in THERMAL_NAMES:
        return None
    return ThermalPacket(
        sequence, x, y, temperature, state,
        time.time() if received_at is None else received_at,
    )


def _parse_named_records(value, count, states, packet_type, sequence, timestamp):
    if count < 0:
        raise ValueError('record count must be non-negative')
    raw_records = [] if value == '' else value.split(';')
    if len(raw_records) != count:
        raise ValueError('record count does not match payload')
    records = []
    identifiers = set()
    for raw in raw_records:
        fields = raw.split(':')
        if len(fields) != 4:
            raise ValueError('record field count is invalid')
        identifier = fields[0]
        if (
            not identifier
            or not all(character.isalnum() or character in '_-' for character in identifier)
            or identifier in identifiers
        ):
            raise ValueError('record id is invalid or duplicated')
        x, y = _parse_finite(fields[1:3])
        state = fields[3].upper()
        if state not in states:
            raise ValueError('record state is unknown')
        identifiers.add(identifier)
        records.append(packet_type(sequence, identifier, x, y, state, timestamp))
    return tuple(records)


def parse_status_packet(line, received_at=None):
    fields = line.strip().split(',')
    if len(fields) not in (10, 16) or fields[0] != 'STATUS':
        return None
    try:
        sequence = _parse_int(fields[1])
        x, y, yaw = _parse_finite(fields[2:5])
        thermal_valid = int(fields[5])
    except ValueError:
        return None
    if thermal_valid not in (0, 1):
        return None
    timestamp = time.time() if received_at is None else received_at
    thermal = None
    if thermal_valid:
        try:
            thermal_x, thermal_y, temperature = _parse_finite(fields[6:9])
            state = int(fields[9])
        except ValueError:
            return None
        if state not in THERMAL_NAMES:
            return None
        thermal = ThermalPacket(
            sequence, thermal_x, thermal_y, temperature, state, timestamp
        )
    human_valid = exit_valid = False
    humans = exits = ()
    if len(fields) == 16:
        try:
            human_valid = bool(int(fields[10]))
            human_count = int(fields[11])
            exit_valid = bool(int(fields[13]))
            exit_count = int(fields[14])
            if fields[10] not in ('0', '1') or fields[13] not in ('0', '1'):
                raise ValueError
            humans = _parse_named_records(
                fields[12], human_count, HUMAN_STATES,
                HumanPacket, sequence, timestamp,
            ) if human_valid else ()
            exits = _parse_named_records(
                fields[15], exit_count, EXIT_STATES,
                ExitPacket, sequence, timestamp,
            ) if exit_valid else ()
            if (not human_valid and (human_count != 0 or fields[12])) or (
                not exit_valid and (exit_count != 0 or fields[15])
            ):
                raise ValueError
        except ValueError:
            return None
    return StatusPacket(
        sequence, x, y, yaw, thermal,
        human_valid, humans, exit_valid, exits, timestamp,
    )


def parse_human_packet(line, received_at=None):
    fields = line.strip().split(',')
    if len(fields) != 6 or fields[0] != 'HUMAN':
        return None
    try:
        sequence = _parse_int(fields[1])
        identifier = fields[2]
        x, y = _parse_finite(fields[3:5])
        state = fields[5].upper()
    except ValueError:
        return None
    if state not in HUMAN_STATES or not _valid_identifier(identifier):
        return None
    return HumanPacket(
        sequence, identifier, x, y, state,
        time.time() if received_at is None else received_at,
    )


def parse_human_remove_packet(line, received_at=None):
    fields = line.strip().split(',')
    if len(fields) != 3 or fields[0] != 'HUMAN_REMOVE':
        return None
    try:
        sequence = _parse_int(fields[1])
    except ValueError:
        return None
    if not _valid_identifier(fields[2]):
        return None
    return HumanRemovePacket(
        sequence, fields[2],
        time.time() if received_at is None else received_at,
    )


def parse_exit_packet(line, received_at=None):
    fields = line.strip().split(',')
    if len(fields) != 6 or fields[0] != 'EXIT':
        return None
    try:
        sequence = _parse_int(fields[1])
        identifier = fields[2]
        x, y = _parse_finite(fields[3:5])
        state = fields[5].upper()
    except ValueError:
        return None
    if state not in EXIT_STATES or not _valid_identifier(identifier):
        return None
    return ExitPacket(
        sequence, identifier, x, y, state,
        time.time() if received_at is None else received_at,
    )


def parse_packet(line, received_at=None):
    prefix = line.strip().split(',', 1)[0]
    if prefix == 'STATUS':
        return parse_status_packet(line, received_at)
    if prefix == 'TEMP':
        return parse_temp_packet(line, received_at)
    if prefix == 'HUMAN':
        return parse_human_packet(line, received_at)
    if prefix == 'HUMAN_REMOVE':
        return parse_human_remove_packet(line, received_at)
    if prefix == 'EXIT':
        return parse_exit_packet(line, received_at)
    if prefix == 'POSE':
        return parse_pose_packet(line, received_at)
    return None


class LatestTelemetry:
    def __init__(self):
        self._lock = threading.Lock()
        self._pose = None
        self._thermal = None
        self._humans = {}
        self._exits = {}
        self._human_valid = False
        self._exit_valid = False
        self._version = 0
        self._expected_sequences = {}

    def update(self, packet):
        warning = None
        packet_type = type(packet).__name__.removesuffix('Packet').upper()
        sequence_type = 'HUMAN' if packet_type == 'HUMANREMOVE' else packet_type
        with self._lock:
            expected = self._expected_sequences.get(sequence_type)
            if expected is not None and packet.sequence != expected:
                warning = (
                    f'{sequence_type} sequence gap: '
                    f'expected {expected}, received {packet.sequence}'
                )
            self._expected_sequences[sequence_type] = packet.sequence + 1
            if isinstance(packet, StatusPacket):
                self._pose = PosePacket(
                    packet.sequence, packet.x, packet.y, packet.yaw,
                    packet.received_at,
                )
                self._thermal = packet.thermal
                self._human_valid = packet.human_valid
                self._humans = {
                    item.identifier: item for item in packet.humans
                } if packet.human_valid else {}
                self._exit_valid = packet.exit_valid
                self._exits = {
                    item.identifier: item for item in packet.exits
                } if packet.exit_valid else {}
            elif isinstance(packet, PosePacket):
                self._pose = packet
            elif isinstance(packet, ThermalPacket):
                self._thermal = packet
            elif isinstance(packet, HumanPacket):
                self._human_valid = True
                self._humans[packet.identifier] = packet
            elif isinstance(packet, HumanRemovePacket):
                self._human_valid = True
                self._humans.pop(packet.identifier, None)
            elif isinstance(packet, ExitPacket):
                self._exit_valid = True
                self._exits[packet.identifier] = packet
            else:
                raise TypeError('unsupported telemetry packet')
            self._version += 1
        return warning

    def snapshot(self):
        with self._lock:
            return (
                self._pose, self._thermal, dict(self._humans),
                self._human_valid, dict(self._exits), self._exit_valid,
                self._version,
            )


# Retain the Stage 5 public name for simple external checks.
LatestPose = LatestTelemetry


class SerialTelemetryReader(threading.Thread):
    def __init__(self, serial_module, port, baud, latest):
        super().__init__(name='lora-telemetry-reader', daemon=True)
        self._serial_module = serial_module
        self._latest = latest
        self._stop_event = threading.Event()
        self._connection = serial_module.Serial(port, baud, timeout=0.5)

    def run(self):
        while not self._stop_event.is_set():
            try:
                raw_line = self._connection.readline()
            except (self._serial_module.SerialException, OSError) as exc:
                if not self._stop_event.is_set():
                    print(f'Serial read failed: {exc}')
                break
            if not raw_line:
                continue
            packet = parse_packet(raw_line.decode('ascii', errors='replace'))
            if packet is None:
                continue
            warning = self._latest.update(packet)
            if warning:
                print(f'WARNING: {warning}')

    def stop(self):
        self._stop_event.set()
        try:
            if self._connection.is_open:
                self._connection.close()
        except (self._serial_module.SerialException, OSError):
            pass
        if self.is_alive():
            self.join(timeout=2.0)


# Retain the Stage 5 public name.
SerialPoseReader = SerialTelemetryReader


def map_world_bounds(map_data):
    cosine = math.cos(map_data.origin_yaw)
    sine = math.sin(map_data.origin_yaw)
    width_m = map_data.width * map_data.resolution
    height_m = map_data.height * map_data.resolution
    corners = []
    for x_map, y_map in (
        (0.0, 0.0), (width_m, 0.0),
        (0.0, height_m), (width_m, height_m),
    ):
        corners.append((
            map_data.origin_x + cosine * x_map - sine * y_map,
            map_data.origin_y + sine * x_map + cosine * y_map,
        ))
    return (
        min(point[0] for point in corners),
        max(point[0] for point in corners),
        min(point[1] for point in corners),
        max(point[1] for point in corners),
    )


def render_telemetry_image(map_data, pose, thermal=None, humans=None, exits=None):
    rendered = map_data.image.convert('RGB').copy()
    draw = ImageDraw.Draw(rendered)
    pose_visible = False
    thermal_visible = False
    if pose is not None:
        pixel_x, pixel_y = world_to_pixel(map_data, pose.x, pose.y)
        pose_visible = pixel_is_in_bounds(map_data, pixel_x, pixel_y)
        if pose_visible:
            end_x, end_y = heading_endpoint(
                map_data, pixel_x, pixel_y, pose.yaw
            )
            radius = 6.0
            draw.ellipse(
                (pixel_x - radius, pixel_y - radius,
                 pixel_x + radius, pixel_y + radius),
                fill='red', outline='white', width=2,
            )
            draw.line((pixel_x, pixel_y, end_x, end_y), fill='red', width=3)
    if thermal is not None:
        pixel_x, pixel_y = world_to_pixel(map_data, thermal.x, thermal.y)
        thermal_visible = pixel_is_in_bounds(map_data, pixel_x, pixel_y)
        if thermal_visible:
            radius = 7.0
            color = THERMAL_COLORS[thermal.state]
            draw.ellipse(
                (pixel_x - radius, pixel_y - radius,
                 pixel_x + radius, pixel_y + radius),
                fill=color, outline='white', width=2,
            )
    for human in (humans or {}).values():
        pixel_x, pixel_y = world_to_pixel(map_data, human.x, human.y)
        if pixel_is_in_bounds(map_data, pixel_x, pixel_y):
            radius = 6.0
            draw.ellipse(
                (pixel_x - radius, pixel_y - radius,
                 pixel_x + radius, pixel_y + radius),
                fill=HUMAN_COLORS[human.state], outline='white', width=2,
            )
    for item in (exits or {}).values():
        pixel_x, pixel_y = world_to_pixel(map_data, item.x, item.y)
        if pixel_is_in_bounds(map_data, pixel_x, pixel_y):
            radius = 5.0
            draw.rectangle(
                (pixel_x - radius, pixel_y - radius,
                 pixel_x + radius, pixel_y + radius),
                fill=EXIT_COLORS[item.state], outline='white', width=1,
            )
    return rendered, pose_visible, thermal_visible


def render_pose_image(map_data, pose):
    rendered, visible, _ = render_telemetry_image(map_data, pose)
    return rendered, visible


def _map_to_world(map_data, x_map, y_map):
    cosine = math.cos(map_data.origin_yaw)
    sine = math.sin(map_data.origin_yaw)
    return (
        map_data.origin_x + cosine * x_map - sine * y_map,
        map_data.origin_y + sine * x_map + cosine * y_map,
    )


def make_demo_status(map_data, sequence, received_at=None):
    phase = ((sequence - 1) % 20) / 19.0
    width_m = map_data.width * map_data.resolution
    height_m = map_data.height * map_data.resolution
    robot_x, robot_y = _map_to_world(
        map_data,
        width_m * (0.15 + 0.70 * phase),
        height_m * (0.50 + 0.20 * math.sin(phase * 2.0 * math.pi)),
    )
    thermal_x, thermal_y = _map_to_world(
        map_data, width_m * 0.68, height_m * 0.62
    )
    cycle = ((sequence - 1) // 3) % 3
    temperatures = (35.0, 45.0, 55.0)
    states = (0, 1, 2)
    timestamp = time.time() if received_at is None else received_at
    thermal = ThermalPacket(
        sequence, thermal_x, thermal_y,
        temperatures[cycle], states[cycle], timestamp,
    )
    humans = ()
    if sequence >= 2:
        human_x, human_y = _map_to_world(
            map_data,
            width_m * (0.30 + 0.02 * min(sequence - 2, 6)),
            height_m * 0.35,
        )
        humans = (HumanPacket(
            sequence, '1', human_x, human_y,
            'CONFIRMED' if sequence < 6 else 'ASSIST_CHECK', timestamp,
        ),)
    exit1_x, exit1_y = _map_to_world(map_data, width_m * 0.10, height_m * 0.15)
    exit2_x, exit2_y = _map_to_world(map_data, width_m * 0.90, height_m * 0.75)
    exits = (
        ExitPacket(sequence, 'EXIT1', exit1_x, exit1_y, 'USABLE', timestamp),
        ExitPacket(
            sequence, 'EXIT2', exit2_x, exit2_y,
            'USABLE' if sequence < 7 else 'BLOCKED', timestamp,
        ),
    )
    return StatusPacket(
        sequence, robot_x, robot_y,
        map_data.origin_yaw + phase * 2.0 * math.pi,
        thermal, True, humans, True, exits, timestamp,
    )


def make_demo_pose(map_data, sequence, received_at=None):
    status = make_demo_status(map_data, sequence, received_at)
    return PosePacket(
        status.sequence, status.x, status.y, status.yaw, status.received_at
    )


class MapPoseViewer:
    def __init__(self, root, map_data, latest, demo=False, display_scale=1.3):
        self.root = root
        self.map_data = map_data
        self.latest = latest
        self.demo = demo
        self.display_scale = display_scale
        self.last_version = -1
        self.last_demo_update = 0.0
        self.demo_sequence = 1

        root.title('LoRa Robot + Thermal + Human + Exit Viewer')
        display_width = round(map_data.width * display_scale)
        display_height = round(map_data.height * display_scale)
        self.canvas = __import__('tkinter').Canvas(
            root, width=display_width, height=display_height,
            highlightthickness=0,
        )
        self.canvas.pack()
        from PIL import ImageTk
        display_image = map_data.image.resize(
            (display_width, display_height), Image.Resampling.NEAREST
        )
        self.map_photo = ImageTk.PhotoImage(display_image)
        self.canvas.create_image(0, 0, anchor='nw', image=self.map_photo)
        self.robot_dot = self.canvas.create_oval(0, 0, 0, 0, state='hidden')
        self.heading = self.canvas.create_line(
            0, 0, 0, 0, fill='red', width=3, arrow='last', state='hidden'
        )
        self.thermal_dot = self.canvas.create_oval(0, 0, 0, 0, state='hidden')
        self.thermal_label = self.canvas.create_text(
            0, 0, anchor='sw', fill='black', state='hidden'
        )
        self.human_markers = {}
        self.exit_markers = {}
        self.info_background = self.canvas.create_rectangle(
            6, 6, 130, 28, fill='white', outline='black'
        )
        self.info_text = self.canvas.create_text(
            9, 8, anchor='nw', fill='black', font=('TkDefaultFont', 8),
            text='Waiting for STATUS...'
        )
        self._resize_info_background()
        self.root.after(100, self._refresh)

    def _resize_info_background(self):
        """Fit the status panel to its current text instead of covering the map."""
        bounds = self.canvas.bbox(self.info_text)
        if bounds is None:
            return
        padding_x = 3
        padding_y = 2
        self.canvas.coords(
            self.info_background,
            bounds[0] - padding_x,
            bounds[1] - padding_y,
            bounds[2] + padding_x,
            bounds[3] + padding_y,
        )

    def _refresh(self):
        if self.demo and time.monotonic() - self.last_demo_update >= 2.0:
            warning = self.latest.update(
                make_demo_status(self.map_data, self.demo_sequence)
            )
            if warning:
                print(f'WARNING: {warning}')
            self.demo_sequence += 1
            self.last_demo_update = time.monotonic()

        (
            pose, thermal, humans, human_valid,
            exits, exit_valid, version,
        ) = self.latest.snapshot()
        if version != self.last_version:
            self._display(pose, thermal, humans, human_valid, exits, exit_valid)
            self.last_version = version
        self.root.after(100, self._refresh)

    def _display(self, pose, thermal, humans, human_valid, exits, exit_valid):
        if pose is not None:
            pixel_x, pixel_y = world_to_pixel(self.map_data, pose.x, pose.y)
            if not pixel_is_in_bounds(self.map_data, pixel_x, pixel_y):
                self.canvas.itemconfigure(self.robot_dot, state='hidden')
                self.canvas.itemconfigure(self.heading, state='hidden')
                print(
                    'WARNING: Robot pose outside map bounds: '
                    f'x={pose.x:.3f}, y={pose.y:.3f}'
                )
            else:
                pixel_x *= self.display_scale
                pixel_y *= self.display_scale
                radius = 6.0
                self.canvas.coords(
                    self.robot_dot, pixel_x - radius, pixel_y - radius,
                    pixel_x + radius, pixel_y + radius,
                )
                self.canvas.itemconfigure(
                    self.robot_dot, fill='red', outline='white', width=2,
                    state='normal',
                )

                end_x, end_y = heading_endpoint(
                    self.map_data, pixel_x, pixel_y, pose.yaw
                )
                self.canvas.coords(
                    self.heading, pixel_x, pixel_y, end_x, end_y
                )
                self.canvas.itemconfigure(self.heading, state='normal')

        self._display_humans(humans if human_valid else {})
        self._display_exits(exits if exit_valid else {})

        if thermal is None:
            self.canvas.itemconfigure(self.thermal_dot, state='hidden')
            self.canvas.itemconfigure(self.thermal_label, state='hidden')
        else:
            pixel_x, pixel_y = world_to_pixel(
                self.map_data, thermal.x, thermal.y
            )
            if not pixel_is_in_bounds(self.map_data, pixel_x, pixel_y):
                self.canvas.itemconfigure(self.thermal_dot, state='hidden')
                self.canvas.itemconfigure(self.thermal_label, state='hidden')
                print(
                    'WARNING: Thermal pose outside map bounds: '
                    f'x={thermal.x:.3f}, y={thermal.y:.3f}'
                )
            else:
                pixel_x *= self.display_scale
                pixel_y *= self.display_scale
                radius = 7.0
                color = THERMAL_COLORS[thermal.state]
                self.canvas.coords(
                    self.thermal_dot, pixel_x - radius, pixel_y - radius,
                    pixel_x + radius, pixel_y + radius,
                )
                self.canvas.itemconfigure(
                    self.thermal_dot, fill=color, outline='white', width=2,
                    state='normal',
                )
                self.canvas.coords(self.thermal_label, pixel_x + 10, pixel_y - 8)
                self.canvas.itemconfigure(
                    self.thermal_label,
                    text=f'{thermal.temperature:.1f} C',
                    state='normal',
                )

        lines = []
        if pose is not None:
            updated = datetime.fromtimestamp(pose.received_at).strftime('%H:%M:%S')
            lines.extend((
                'Robot',
                f'X: {pose.x:.3f} m',
                f'Y: {pose.y:.3f} m',
                f'Yaw: {pose.yaw:.3f} rad',
                f'Sequence: {pose.sequence}',
                f'Last update: {updated}',
            ))
        if thermal is None:
            lines.extend(('', 'Thermal: unavailable'))
        else:
            lines.extend((
                '',
                f'Thermal: {thermal.temperature:.1f} C '
                f'({THERMAL_NAMES[thermal.state]})',
                f'Location: {thermal.x:.3f}, {thermal.y:.3f} m',
            ))
        lines.append(
            f'Human: {len(humans)}' if human_valid else 'Human: unavailable'
        )
        if human_valid:
            lines.extend(
                f'  {identifier}: {item.state}'
                for identifier, item in sorted(humans.items())
            )
        lines.append(
            f'Exit: {len(exits)}' if exit_valid else 'Exit: unavailable'
        )
        if exit_valid:
            lines.extend(
                f'  {identifier}: {item.state}'
                for identifier, item in sorted(exits.items())
            )
        self.canvas.itemconfigure(self.info_text, text='\n'.join(lines))
        self._resize_info_background()

    def _display_humans(self, humans):
        for identifier in set(self.human_markers) - set(humans):
            for canvas_id in self.human_markers.pop(identifier):
                self.canvas.delete(canvas_id)
        for identifier, item in humans.items():
            pixel_x, pixel_y = world_to_pixel(self.map_data, item.x, item.y)
            if not pixel_is_in_bounds(self.map_data, pixel_x, pixel_y):
                if identifier in self.human_markers:
                    for canvas_id in self.human_markers.pop(identifier):
                        self.canvas.delete(canvas_id)
                print(
                    'WARNING: Human pose outside map bounds: '
                    f'id={identifier}, x={item.x:.3f}, y={item.y:.3f}'
                )
                continue
            pixel_x *= self.display_scale
            pixel_y *= self.display_scale
            if identifier not in self.human_markers:
                self.human_markers[identifier] = (
                    self.canvas.create_oval(0, 0, 0, 0),
                    self.canvas.create_text(0, 0, anchor='sw'),
                )
            dot, label = self.human_markers[identifier]
            radius = 6.0
            self.canvas.coords(
                dot, pixel_x - radius, pixel_y - radius,
                pixel_x + radius, pixel_y + radius,
            )
            self.canvas.itemconfigure(
                dot, fill=HUMAN_COLORS[item.state], outline='white', width=2
            )
            self.canvas.coords(label, pixel_x + 9, pixel_y - 7)
            self.canvas.itemconfigure(label, text=f'H{identifier}: {item.state}')

    def _display_exits(self, exits):
        for identifier in set(self.exit_markers) - set(exits):
            for canvas_id in self.exit_markers.pop(identifier):
                self.canvas.delete(canvas_id)
        for identifier, item in exits.items():
            pixel_x, pixel_y = world_to_pixel(self.map_data, item.x, item.y)
            if not pixel_is_in_bounds(self.map_data, pixel_x, pixel_y):
                if identifier in self.exit_markers:
                    for canvas_id in self.exit_markers.pop(identifier):
                        self.canvas.delete(canvas_id)
                print(
                    'WARNING: Exit pose outside map bounds: '
                    f'id={identifier}, x={item.x:.3f}, y={item.y:.3f}'
                )
                continue
            pixel_x *= self.display_scale
            pixel_y *= self.display_scale
            if identifier not in self.exit_markers:
                self.exit_markers[identifier] = (
                    self.canvas.create_rectangle(0, 0, 0, 0),
                    self.canvas.create_text(0, 0, anchor='sw'),
                )
            marker, label = self.exit_markers[identifier]
            radius = 5.0
            self.canvas.coords(
                marker, pixel_x - radius, pixel_y - radius,
                pixel_x + radius, pixel_y + radius,
            )
            self.canvas.itemconfigure(
                marker, fill=EXIT_COLORS[item.state], outline='white', width=1
            )
            self.canvas.coords(label, pixel_x + 8, pixel_y - 6)
            self.canvas.itemconfigure(
                label,
                text=f'{identifier.lower()}_status: {item.state.lower()}',
            )


def parse_args(argv=None):
    parser = argparse.ArgumentParser(
        description='Display LoRa robot/thermal/human/exit data on a ROS map.'
    )
    parser.add_argument('--map', required=True, help='ROS map YAML path')
    parser.add_argument('--port', help='Serial port, e.g. /dev/ttyUSB0 or COM3')
    parser.add_argument('--baud', type=int, default=9600, help='UART baud rate')
    parser.add_argument(
        '--display-scale', type=float, default=1.3,
        help='Map display scale (default: 1.3)',
    )
    parser.add_argument(
        '--demo', action='store_true',
        help='Animate robot, thermal, human, and exit states without serial',
    )
    args = parser.parse_args(argv)
    if not args.demo and not args.port:
        parser.error('--port is required unless --demo is used')
    if args.baud <= 0:
        parser.error('--baud must be positive')
    if args.display_scale <= 0:
        parser.error('--display-scale must be positive')
    return args


def main():
    args = parse_args()
    try:
        map_data = load_map(args.map)
    except (OSError, ValueError, yaml.YAMLError) as exc:
        print(f'Failed to load map {args.map}: {exc}')
        return 1

    latest = LatestTelemetry()
    reader = None
    if not args.demo:
        try:
            import serial
            reader = SerialTelemetryReader(serial, args.port, args.baud, latest)
        except ImportError:
            print('pyserial is required for serial mode')
            return 1
        except (serial.SerialException, OSError) as exc:
            print(f'Failed to open serial port {args.port}: {exc}')
            return 1
        reader.start()

    try:
        import tkinter as tk
    except ImportError:
        print('tkinter is required for the viewer window')
        if reader is not None:
            reader.stop()
        return 1

    try:
        root = tk.Tk()
        MapPoseViewer(
            root, map_data, latest, demo=args.demo,
            display_scale=args.display_scale,
        )
        root.mainloop()
    except KeyboardInterrupt:
        pass
    except tk.TclError as exc:
        print(f'Failed to start GUI: {exc}')
        return 1
    finally:
        if reader is not None:
            reader.stop()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
