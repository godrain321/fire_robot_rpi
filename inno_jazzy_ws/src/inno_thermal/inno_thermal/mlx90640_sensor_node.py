"""Publish raw MLX90640 temperatures and a short thermal direction arc."""

from __future__ import annotations

import ctypes
import os
from pathlib import Path

import numpy as np
from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Float32MultiArray, MultiArrayDimension

from inno_thermal.thermal_geometry import (
    THERMAL_HEIGHT,
    THERMAL_WIDTH,
    apply_orientation,
    compute_column_max,
    project_columns_to_arc,
)


SUPPORTED_REFRESH_RATES_HZ = (1, 2, 4, 8, 16, 32, 64)
FRAME_SAMPLE_COUNT = THERMAL_HEIGHT * THERMAL_WIDTH


class NativeMlx90640:
    """Small ctypes adapter for the repository's existing C++ wrapper."""

    def __init__(self, library_path: Path, i2c_address: int, refresh_rate_hz: int):
        self.library_path = Path(library_path).expanduser().resolve()
        self.i2c_address = int(i2c_address)
        self.refresh_rate_hz = int(refresh_rate_hz)
        try:
            self._library = ctypes.CDLL(str(self.library_path))
        except OSError as exc:
            raise RuntimeError(
                f"failed to load MLX90640 native library {self.library_path}: {exc}"
            ) from exc
        try:
            initialize = self._library.InnoMlx90640_Init
            read_frame = self._library.InnoMlx90640_ReadFrame
            close = self._library.InnoMlx90640_Close
        except AttributeError as exc:
            raise RuntimeError(
                f"{self.library_path} does not contain the inno_thermal status bridge; "
                "rebuild it with scripts/build_native_driver.sh"
            ) from exc
        initialize.argtypes = [ctypes.c_int, ctypes.c_int]
        initialize.restype = ctypes.c_int
        read_frame.argtypes = [
            ctypes.c_int,
            ctypes.POINTER(ctypes.c_float),
        ]
        read_frame.restype = ctypes.c_int
        close.argtypes = []
        close.restype = None
        self._buffer = (ctypes.c_float * FRAME_SAMPLE_COUNT)()
        status = initialize(self.i2c_address, self.refresh_rate_hz)
        if status != 0:
            raise RuntimeError(f"MLX90640 initialization failed with status {status}")

    def read_frame(self) -> np.ndarray:
        status = self._library.InnoMlx90640_ReadFrame(
            self.i2c_address, self._buffer
        )
        if status != 0:
            raise RuntimeError(f"MLX90640 frame read failed with status {status}")
        raw = np.ctypeslib.as_array(self._buffer)
        return np.array(raw, dtype=np.float32, copy=True).reshape(
            THERMAL_HEIGHT, THERMAL_WIDTH
        )

    def close(self) -> None:
        if self._library is not None:
            self._library.InnoMlx90640_Close()
        self._buffer = None


def _library_candidates(configured_path: str) -> list[Path]:
    candidates: list[Path] = []
    if configured_path.strip():
        candidates.append(Path(configured_path))
    environment_path = os.environ.get("INNO_THERMAL_MLX90640_LIBRARY", "").strip()
    if environment_path:
        candidates.append(Path(environment_path))

    try:
        share = Path(get_package_share_directory("inno_thermal"))
        candidates.append(share / "native" / "libmlx90640.so")
    except (LookupError, IndexError):
        pass

    source_file = Path(__file__).resolve()
    for parent in source_file.parents:
        candidates.append(parent / "native/libmlx90640.so")
    return candidates


def resolve_native_library(configured_path: str) -> Path:
    checked: list[str] = []
    for candidate in _library_candidates(configured_path):
        resolved = candidate.expanduser().resolve()
        if str(resolved) in checked:
            continue
        checked.append(str(resolved))
        if resolved.is_file():
            return resolved
    locations = "\n  - ".join(checked) if checked else "(none)"
    raise FileNotFoundError(
        "libmlx90640.so was not found. Run scripts/build_native_driver.sh or "
        "set native_library_path / INNO_THERMAL_MLX90640_LIBRARY. Checked:\n  - "
        + locations
    )


class Mlx90640SensorNode(Node):
    def __init__(self) -> None:
        super().__init__("mlx90640_sensor_node")
        self.declare_parameter("i2c_address", 0x33)
        self.declare_parameter("frame_id", "thermal_camera_link")
        self.declare_parameter("horizontal_fov_deg", 110.0)
        self.declare_parameter("vertical_fov_deg", 75.0)
        self.declare_parameter("projection_distance_m", 0.15)
        self.declare_parameter("refresh_rate_hz", 8)
        self.declare_parameter("publish_rate_hz", 4.0)
        # Hardware calibration: the native sensor order is horizontally
        # mirrored relative to physical left/right on the installed camera.
        self.declare_parameter("flip_horizontal", True)
        self.declare_parameter("flip_vertical", False)
        self.declare_parameter("rotate_180", False)
        self.declare_parameter("native_library_path", "")

        self.i2c_address = int(self.get_parameter("i2c_address").value)
        self.frame_id = str(self.get_parameter("frame_id").value)
        self.horizontal_fov_deg = float(
            self.get_parameter("horizontal_fov_deg").value
        )
        self.vertical_fov_deg = float(self.get_parameter("vertical_fov_deg").value)
        self.projection_distance_m = float(
            self.get_parameter("projection_distance_m").value
        )
        self.refresh_rate_hz = int(self.get_parameter("refresh_rate_hz").value)
        self.publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)
        self.flip_horizontal = bool(self.get_parameter("flip_horizontal").value)
        self.flip_vertical = bool(self.get_parameter("flip_vertical").value)
        self.rotate_180 = bool(self.get_parameter("rotate_180").value)
        configured_library = str(self.get_parameter("native_library_path").value)
        self._validate_parameters()

        library_path = resolve_native_library(configured_library)
        self.get_logger().info(f"Loading MLX90640 driver: {library_path}")
        self._sensor = NativeMlx90640(
            library_path, self.i2c_address, self.refresh_rate_hz
        )
        self._image_publisher = self.create_publisher(Image, "/thermal/image", 10)
        self._column_publisher = self.create_publisher(
            Float32MultiArray, "/thermal/column_max", 10
        )
        self._arc_publisher = self.create_publisher(
            PointCloud2, "/thermal/arc_points", 10
        )
        self._timer = self.create_timer(1.0 / self.publish_rate_hz, self._publish_frame)

    def _validate_parameters(self) -> None:
        if not 0 <= self.i2c_address <= 0x7F:
            raise ValueError("i2c_address must be a 7-bit address in [0, 127]")
        if self.refresh_rate_hz not in SUPPORTED_REFRESH_RATES_HZ:
            raise ValueError(
                "refresh_rate_hz must be one of "
                f"{SUPPORTED_REFRESH_RATES_HZ}; got {self.refresh_rate_hz}"
            )
        if not np.isfinite(self.publish_rate_hz) or self.publish_rate_hz <= 0.0:
            raise ValueError("publish_rate_hz must be finite and positive")
        if self.publish_rate_hz > self.refresh_rate_hz:
            raise ValueError("publish_rate_hz cannot exceed refresh_rate_hz")
        if not np.isfinite(self.vertical_fov_deg) or not 0.0 < self.vertical_fov_deg < 180.0:
            raise ValueError("vertical_fov_deg must be finite and in (0, 180)")
        # Geometry helpers validate horizontal FOV and distance consistently.
        project_columns_to_arc(
            np.zeros(THERMAL_WIDTH, dtype=np.float32),
            self.horizontal_fov_deg,
            self.projection_distance_m,
        )

    def _publish_frame(self) -> None:
        try:
            raw = self._sensor.read_frame()
            if raw.shape != (THERMAL_HEIGHT, THERMAL_WIDTH):
                raise ValueError(f"native driver returned unexpected shape {raw.shape}")
            # A single invalid raw sensor sample invalidates this frame. This
            # avoids publishing an apparently complete image with hidden gaps.
            if not np.isfinite(raw).all():
                raise ValueError("raw frame contains NaN or infinite temperatures")
            temperatures = apply_orientation(
                raw,
                flip_horizontal=self.flip_horizontal,
                flip_vertical=self.flip_vertical,
                rotate_180=self.rotate_180,
            )
            column_max = compute_column_max(temperatures)
            points = project_columns_to_arc(
                column_max, self.horizontal_fov_deg, self.projection_distance_m
            )
            stamp = self.get_clock().now().to_msg()
            self._image_publisher.publish(self._make_image(temperatures, stamp))
            self._column_publisher.publish(self._make_column_max(column_max))
            self._arc_publisher.publish(self._make_point_cloud(points, stamp))
        except Exception as exc:  # keep the timer/node alive after sensor errors
            self.get_logger().error(f"MLX90640 frame read/publish failed; retrying: {exc}")

    def _make_image(self, temperatures: np.ndarray, stamp) -> Image:
        contiguous = np.ascontiguousarray(temperatures, dtype=np.float32)
        message = Image()
        message.header.stamp = stamp
        message.header.frame_id = self.frame_id
        message.height = THERMAL_HEIGHT
        message.width = THERMAL_WIDTH
        message.encoding = "32FC1"
        message.is_bigendian = False
        message.step = THERMAL_WIDTH * np.dtype(np.float32).itemsize
        message.data = contiguous.tobytes(order="C")
        return message

    @staticmethod
    def _make_column_max(column_max: np.ndarray) -> Float32MultiArray:
        message = Float32MultiArray()
        dimension = MultiArrayDimension()
        dimension.label = "thermal_columns"
        dimension.size = THERMAL_WIDTH
        dimension.stride = THERMAL_WIDTH
        message.layout.dim = [dimension]
        message.data = [float(value) for value in column_max]
        return message

    def _make_point_cloud(self, points: np.ndarray, stamp) -> PointCloud2:
        header = self._make_header(stamp)
        fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name="intensity", offset=12, datatype=PointField.FLOAT32, count=1),
        ]
        return point_cloud2.create_cloud(header, fields, points.tolist())

    def _make_header(self, stamp):
        from std_msgs.msg import Header

        header = Header()
        header.stamp = stamp
        header.frame_id = self.frame_id
        return header

    def destroy_node(self):
        if getattr(self, "_timer", None) is not None:
            self._timer.cancel()
        if getattr(self, "_sensor", None) is not None:
            self._sensor.close()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = Mlx90640SensorNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
