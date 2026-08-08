#!/usr/bin/env python3
"""Professional Tk dashboard for the DFRobot C4001/SEN0610 sensor.

ROS callbacks never touch Tk directly.  They update :class:`TelemetryStore`,
which is safe to read from the Tk main thread.  The data model and chart
helpers intentionally remain importable on machines without ROS, Tk, or
Pillow so they can be unit tested in a headless environment.
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
from enum import Enum
import math
import threading
import time
from typing import Callable, Deque, Iterable, Optional, Sequence, Tuple


try:  # Tk may be absent in a headless test image.
    import tkinter as tk
except ImportError:  # pragma: no cover - depends on the target OS image
    tk = None  # type: ignore[assignment]

try:  # Pillow supplies the antialiased background; the UI has a Tk fallback.
    from PIL import Image, ImageDraw, ImageTk
except ImportError:  # pragma: no cover - exercised only without Pillow
    Image = None  # type: ignore[assignment]
    ImageDraw = None  # type: ignore[assignment]
    ImageTk = None  # type: ignore[assignment]

try:  # Keep pure helpers importable when ROS 2 is not sourced/installed.
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
    from std_msgs.msg import Bool, Float32, String, UInt32
except ImportError:  # pragma: no cover - covered indirectly by headless tests
    rclpy = None  # type: ignore[assignment]
    Node = object  # type: ignore[assignment,misc]
    DurabilityPolicy = HistoryPolicy = QoSProfile = ReliabilityPolicy = None  # type: ignore[assignment,misc]
    Bool = Float32 = String = UInt32 = None  # type: ignore[assignment,misc]


ROS_AVAILABLE = rclpy is not None
MAX_DISTANCE_M = 12.0
HISTORY_WINDOW_SEC = 30.0
DEFAULT_STALE_TIMEOUT_SEC = 3.0
FILTERED_PRESENCE_TOPIC = "/mmwave/filtered_presence"
FILTERED_DISTANCE_TOPIC = "/mmwave/filtered_distance_m"
FILTERED_SPEED_TOPIC = "/mmwave/filtered_speed_mps"
RAW_ENERGY_TOPIC = "/mmwave/raw/energy_raw"
MOTION_ACTIVITY_TOPIC = "/mmwave/motion_activity"


class MobilityState(str, Enum):
    """Classifier states shared by the driver, console, and GUI."""

    NO_TARGET = "NO_TARGET"
    MOVING = "MOVING"
    STILL_MONITOR = "STILL_MONITOR"
    ASSIST_CHECK = "ASSIST_CHECK"
    ROBOT_MOVING = "ROBOT_MOVING"
    SENSOR_OFFLINE = "SENSOR_OFFLINE"


@dataclass(frozen=True)
class MobilityPresentation:
    title: str
    subtitle: str
    detail: str
    color: str


MOBILITY_PRESENTATIONS = {
    MobilityState.NO_TARGET: MobilityPresentation(
        "감지 대상 없음",
        "NO TARGET",
        "대상이 감지되면 움직임 상태를 분석합니다.",
        "#7894A8",
    ),
    MobilityState.MOVING: MobilityPresentation(
        "움직임 감지",
        "MOVING",
        "자력 이동 가능성이 있습니다.",
        "#34E3B3",
    ),
    MobilityState.STILL_MONITOR: MobilityPresentation(
        "움직임 관찰 중",
        "MONITORING",
        "반복 움직임 신호를 확인하고 있습니다.",
        "#F7B955",
    ),
    MobilityState.ASSIST_CHECK: MobilityPresentation(
        "구조 확인 필요",
        "ASSIST CHECK",
        "장시간 미동 없음 · 현장 확인이 필요합니다.",
        "#FF5E72",
    ),
    MobilityState.ROBOT_MOVING: MobilityPresentation(
        "판정 보류",
        "ROBOT IN MOTION",
        "로봇 이동 중에는 상대속도 판정을 보류합니다.",
        "#5BAAFF",
    ),
    MobilityState.SENSOR_OFFLINE: MobilityPresentation(
        "센서 오프라인",
        "SENSOR OFFLINE",
        "UART 연결과 센서 전원을 확인하세요.",
        "#FF5E72",
    ),
}


def normalize_mobility_state(value: object) -> MobilityState:
    """Normalize a classifier string without ever throwing in the GUI loop."""

    if isinstance(value, MobilityState):
        return value
    normalized = str(value or "").strip().upper().replace("-", "_").replace(" ", "_")
    aliases = {
        "": MobilityState.NO_TARGET,
        "NONE": MobilityState.NO_TARGET,
        "NO_PERSON": MobilityState.NO_TARGET,
        "TARGET_MOVING": MobilityState.MOVING,
        "STILL": MobilityState.STILL_MONITOR,
        "MONITOR": MobilityState.STILL_MONITOR,
        "NEEDS_ASSISTANCE": MobilityState.ASSIST_CHECK,
        "ASSIST": MobilityState.ASSIST_CHECK,
        "PAUSED": MobilityState.ROBOT_MOVING,
        "OFFLINE": MobilityState.SENSOR_OFFLINE,
        "ERROR": MobilityState.SENSOR_OFFLINE,
    }
    if normalized in aliases:
        return aliases[normalized]
    try:
        return MobilityState(normalized)
    except ValueError:
        return MobilityState.NO_TARGET


def normalize_sensor_state(value: object) -> str:
    """Return one of ONLINE/CONNECTING/OFFLINE/ERROR for display logic."""

    normalized = str(value or "").strip().upper()
    if normalized.startswith(("ONLINE", "RUNNING", "READY")):
        return "ONLINE"
    if normalized.startswith(("ERROR", "FAULT")):
        return "ERROR"
    if normalized.startswith(("OFFLINE", "DISCONNECTED", "STOPPED")):
        return "OFFLINE"
    return "CONNECTING"


def finite_float(value: object, *, minimum: Optional[float] = None) -> Optional[float]:
    """Convert a value to finite float, optionally rejecting low values."""

    try:
        number = float(value)
    except (TypeError, ValueError):
        return None
    if not math.isfinite(number) or (minimum is not None and number < minimum):
        return None
    return number


def format_distance_m(value: object, *, include_unit: bool = True) -> str:
    """Format a valid target distance without implying centimetre precision."""

    distance = finite_float(value)
    if distance is None or distance <= 0.0:
        return "—"
    suffix = " m" if include_unit else ""
    return f"{distance:.1f}{suffix}"


def format_duration(seconds: object) -> str:
    value = finite_float(seconds, minimum=0.0) or 0.0
    whole = int(value)
    minutes, secs = divmod(whole, 60)
    if minutes >= 60:
        hours, minutes = divmod(minutes, 60)
        return f"{hours:02d}:{minutes:02d}:{secs:02d}"
    return f"{minutes:02d}:{secs:02d}"


@dataclass(frozen=True)
class HistoryPoint:
    timestamp: float
    value: float


@dataclass(frozen=True)
class TelemetrySnapshot:
    timestamp: float
    presence: bool
    distance_m: Optional[float]
    speed_mps: Optional[float]
    energy_raw: Optional[int]
    motion_activity_percent: float
    still_duration_sec: float
    sensor_state: str
    mobility_state: MobilityState
    mobility_duration_sec: float
    online: bool
    last_rx_age_sec: Optional[float]
    distance_history: Tuple[HistoryPoint, ...]
    speed_history: Tuple[HistoryPoint, ...]
    energy_history: Tuple[HistoryPoint, ...]
    motion_history: Tuple[HistoryPoint, ...]


class TelemetryStore:
    """Thread-safe, bounded telemetry state shared by ROS and Tk."""

    def __init__(
        self,
        *,
        clock: Callable[[], float] = time.monotonic,
        history_window_sec: float = HISTORY_WINDOW_SEC,
        stale_timeout_sec: float = DEFAULT_STALE_TIMEOUT_SEC,
    ) -> None:
        self._clock = clock
        self._history_window_sec = max(1.0, float(history_window_sec))
        self._stale_timeout_sec = max(0.25, float(stale_timeout_sec))
        self._lock = threading.RLock()

        self._presence = False
        self._distance_m: Optional[float] = None
        self._speed_mps: Optional[float] = None
        self._energy_raw: Optional[int] = None
        self._motion_activity_percent = 0.0
        self._still_duration_sec = 0.0
        self._sensor_state = "CONNECTING"
        self._reported_mobility: Optional[MobilityState] = None
        self._last_rx_at: Optional[float] = None
        self._effective_mobility = MobilityState.SENSOR_OFFLINE
        self._effective_mobility_since = self._clock()

        self._distance_history: Deque[HistoryPoint] = deque()
        self._speed_history: Deque[HistoryPoint] = deque()
        self._energy_history: Deque[HistoryPoint] = deque()
        self._motion_history: Deque[HistoryPoint] = deque()

    def _touch(self, now: float) -> None:
        self._last_rx_at = now

    def _append(self, series: Deque[HistoryPoint], value: float, now: float) -> None:
        series.append(HistoryPoint(now, value))
        self._prune_one(series, now)

    def _prune_one(self, series: Deque[HistoryPoint], now: float) -> None:
        cutoff = now - self._history_window_sec
        while series and series[0].timestamp < cutoff:
            series.popleft()

    def _prune_all(self, now: float) -> None:
        self._prune_one(self._distance_history, now)
        self._prune_one(self._speed_history, now)
        self._prune_one(self._energy_history, now)
        self._prune_one(self._motion_history, now)

    def update_presence(self, value: object) -> None:
        now = self._clock()
        with self._lock:
            self._presence = bool(value)
            self._touch(now)

    def update_distance(self, value: object) -> None:
        now = self._clock()
        distance = finite_float(value)
        # The driver publishes 0.0 as the no-target sentinel. Treat it as
        # missing data so CLEAR periods cannot draw a false line at 0 m.
        if distance is not None and distance <= 0.0:
            distance = None
        with self._lock:
            self._distance_m = distance
            if distance is not None and self._presence:
                self._append(self._distance_history, distance, now)
            self._touch(now)

    def update_speed(self, value: object) -> None:
        now = self._clock()
        speed = finite_float(value)
        with self._lock:
            self._speed_mps = speed
            if speed is not None:
                self._append(self._speed_history, speed, now)
            self._touch(now)

    def update_energy(self, value: object) -> None:
        now = self._clock()
        energy = finite_float(value, minimum=0.0)
        with self._lock:
            self._energy_raw = int(energy) if energy is not None else None
            if energy is not None:
                self._append(self._energy_history, energy, now)
            self._touch(now)

    def update_motion_activity(self, value: object) -> None:
        now = self._clock()
        activity = finite_float(value, minimum=0.0)
        if activity is None:
            return
        activity = min(100.0, activity)
        with self._lock:
            self._motion_activity_percent = activity
            self._append(self._motion_history, activity, now)
            self._touch(now)

    def update_still_duration(self, value: object) -> None:
        duration = finite_float(value, minimum=0.0)
        with self._lock:
            self._still_duration_sec = duration or 0.0

    def update_sensor_state(self, value: object) -> None:
        now = self._clock()
        with self._lock:
            self._sensor_state = normalize_sensor_state(value)
            self._touch(now)

    def update_mobility_state(self, value: object) -> None:
        state = normalize_mobility_state(value)
        with self._lock:
            self._reported_mobility = state

    def _select_effective_mobility(self, online: bool) -> MobilityState:
        if not online:
            return MobilityState.SENSOR_OFFLINE
        if not self._presence:
            return MobilityState.NO_TARGET
        if self._reported_mobility in (None, MobilityState.NO_TARGET):
            return MobilityState.STILL_MONITOR
        return self._reported_mobility

    def snapshot(self) -> TelemetrySnapshot:
        now = self._clock()
        with self._lock:
            self._prune_all(now)
            age = None if self._last_rx_at is None else max(0.0, now - self._last_rx_at)
            fresh = age is not None and age <= self._stale_timeout_sec
            online = fresh and self._sensor_state == "ONLINE"
            effective = self._select_effective_mobility(online)
            if effective != self._effective_mobility:
                self._effective_mobility = effective
                self._effective_mobility_since = now
            mobility_duration = max(0.0, now - self._effective_mobility_since)
            return TelemetrySnapshot(
                timestamp=now,
                presence=self._presence and online,
                distance_m=self._distance_m,
                speed_mps=self._speed_mps,
                energy_raw=self._energy_raw,
                motion_activity_percent=self._motion_activity_percent,
                still_duration_sec=self._still_duration_sec,
                sensor_state=self._sensor_state if fresh else "OFFLINE",
                mobility_state=effective,
                mobility_duration_sec=mobility_duration,
                online=online,
                last_rx_age_sec=age,
                distance_history=tuple(self._distance_history),
                speed_history=tuple(self._speed_history),
                energy_history=tuple(self._energy_history),
                motion_history=tuple(self._motion_history),
            )


def chart_coordinates(
    samples: Sequence[HistoryPoint],
    *,
    now: float,
    window_sec: float,
    value_min: float,
    value_max: float,
    x: float,
    y: float,
    width: float,
    height: float,
) -> Tuple[Tuple[float, float], ...]:
    """Map timestamp/value pairs into a chart rectangle, with clipping."""

    if window_sec <= 0.0 or value_max <= value_min or width <= 0.0 or height <= 0.0:
        return ()
    start = now - window_sec
    points = []
    for sample in samples:
        if sample.timestamp < start or sample.timestamp > now:
            continue
        time_ratio = (sample.timestamp - start) / window_sec
        value_ratio = (sample.value - value_min) / (value_max - value_min)
        value_ratio = max(0.0, min(1.0, value_ratio))
        points.append((x + time_ratio * width, y + (1.0 - value_ratio) * height))
    return tuple(points)


def nice_upper_bound(values: Iterable[float], *, minimum: float = 1.0) -> float:
    """Return a stable 1/2/5-based chart upper bound."""

    maximum = max((float(value) for value in values if math.isfinite(float(value))), default=0.0)
    maximum = max(maximum, minimum)
    exponent = 10.0 ** math.floor(math.log10(maximum))
    scaled = maximum / exponent
    step = 1.0 if scaled <= 1.0 else 2.0 if scaled <= 2.0 else 5.0 if scaled <= 5.0 else 10.0
    return step * exponent


if ROS_AVAILABLE:

    class MmWaveGuiNode(Node):
        """ROS-only adapter that feeds the GUI's thread-safe store."""

        def __init__(self, store: TelemetryStore) -> None:
            super().__init__("mmwave_visualizer")
            self._store = store
            # The UART driver deliberately latches its current reading/state.
            # Matching its QoS lets a GUI opened later immediately receive
            # ONLINE instead of waiting for a sensor-state transition.
            driver_qos = QoSProfile(
                history=HistoryPolicy.KEEP_LAST,
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            )
            self.create_subscription(
                Bool, FILTERED_PRESENCE_TOPIC, self._on_presence, driver_qos
            )
            self.create_subscription(
                Float32, FILTERED_DISTANCE_TOPIC, self._on_distance, driver_qos
            )
            self.create_subscription(
                Float32, FILTERED_SPEED_TOPIC, self._on_speed, driver_qos
            )
            self.create_subscription(
                UInt32, RAW_ENERGY_TOPIC, self._on_energy, driver_qos
            )
            self.create_subscription(
                Float32, MOTION_ACTIVITY_TOPIC, self._on_motion_activity, driver_qos
            )
            self.create_subscription(
                String, "/mmwave/sensor_state", self._on_sensor_state, driver_qos
            )
            self.create_subscription(
                String, "/mmwave/mobility_state", self._on_mobility, 10
            )
            self.create_subscription(
                Float32,
                "/mmwave/still_duration_sec",
                self._on_still_duration,
                10,
            )

        def _on_presence(self, message: Bool) -> None:
            self._store.update_presence(message.data)

        def _on_distance(self, message: Float32) -> None:
            self._store.update_distance(message.data)

        def _on_speed(self, message: Float32) -> None:
            self._store.update_speed(message.data)

        def _on_energy(self, message: UInt32) -> None:
            self._store.update_energy(message.data)

        def _on_motion_activity(self, message: Float32) -> None:
            self._store.update_motion_activity(message.data)

        def _on_sensor_state(self, message: String) -> None:
            self._store.update_sensor_state(message.data)

        def _on_mobility(self, message: String) -> None:
            self._store.update_mobility_state(message.data)

        def _on_still_duration(self, message: Float32) -> None:
            self._store.update_still_duration(message.data)


class MmWaveDashboard:
    """Responsive 1280x720 canvas dashboard."""

    BASE_WIDTH = 1280.0
    BASE_HEIGHT = 720.0
    FONT_FAMILY = "Noto Sans CJK KR"
    MONO_FAMILY = "DejaVu Sans Mono"

    BG_TOP = "#06111F"
    BG_BOTTOM = "#091B2A"
    CARD = "#0B2030"
    CARD_EDGE = "#16394B"
    GRID = "#173C4D"
    WHITE = "#EAF9FF"
    MUTED = "#7896A9"
    CYAN = "#27DDF4"
    GREEN = "#34E3B3"
    AMBER = "#F7B955"
    RED = "#FF5E72"
    BLUE = "#5BAAFF"

    def __init__(
        self,
        root: "tk.Tk",
        store: TelemetryStore,
        *,
        on_close: Optional[Callable[[], None]] = None,
        should_close: Optional[Callable[[], bool]] = None,
    ) -> None:
        if tk is None:  # pragma: no cover - constructor cannot run without Tk
            raise RuntimeError("Tkinter is required to run the mmWave visualizer")
        self.root = root
        self.store = store
        self._on_close = on_close
        self._should_close = should_close
        self._closing = False
        self._after_id: Optional[str] = None
        self._background_photo = None
        self._background_size: Tuple[int, int] = (0, 0)

        self.root.title("C4001 Life-Sign Monitor")
        self.root.geometry("1280x720")
        self.root.minsize(1024, 576)
        self.root.configure(bg=self.BG_TOP)
        self.root.protocol("WM_DELETE_WINDOW", self.close)
        self.root.bind("<F11>", self._toggle_fullscreen)
        self.root.bind("<Escape>", self._leave_fullscreen)

        self.canvas = tk.Canvas(
            self.root,
            bg=self.BG_TOP,
            highlightthickness=0,
            bd=0,
        )
        self.canvas.pack(fill="both", expand=True)
        self._tick()

    def _toggle_fullscreen(self, _event: object = None) -> None:
        current = bool(self.root.attributes("-fullscreen"))
        self.root.attributes("-fullscreen", not current)

    def _leave_fullscreen(self, _event: object = None) -> None:
        self.root.attributes("-fullscreen", False)

    def close(self) -> None:
        if self._closing:
            return
        self._closing = True
        if self._after_id is not None:
            try:
                self.root.after_cancel(self._after_id)
            except tk.TclError:
                pass
            self._after_id = None
        if self._on_close is not None:
            self._on_close()
        try:
            self.root.quit()
            self.root.destroy()
        except tk.TclError:
            pass

    def run(self) -> None:
        self.root.mainloop()

    def _tick(self) -> None:
        if self._closing:
            return
        if self._should_close is not None and self._should_close():
            self.close()
            return
        try:
            self._render(self.store.snapshot())
            self._after_id = self.root.after(100, self._tick)
        except KeyboardInterrupt:
            self.close()
        except tk.TclError:
            self._closing = True

    def _font(self, size: float, *, bold: bool = False, mono: bool = False) -> tuple:
        scale = min(self.canvas.winfo_width() / self.BASE_WIDTH, self.canvas.winfo_height() / self.BASE_HEIGHT)
        family = self.MONO_FAMILY if mono else self.FONT_FAMILY
        return (family, max(8, int(round(size * max(0.75, scale)))), "bold" if bold else "normal")

    def _xy(self, x: float, y: float) -> Tuple[float, float]:
        return (
            x * self.canvas.winfo_width() / self.BASE_WIDTH,
            y * self.canvas.winfo_height() / self.BASE_HEIGHT,
        )

    def _box(self, x1: float, y1: float, x2: float, y2: float) -> Tuple[float, float, float, float]:
        left, top = self._xy(x1, y1)
        right, bottom = self._xy(x2, y2)
        return left, top, right, bottom

    def _rounded_rectangle(
        self,
        x1: float,
        y1: float,
        x2: float,
        y2: float,
        *,
        radius: float = 16.0,
        fill: str,
        outline: str = "",
        width: float = 1.0,
    ) -> None:
        left, top, right, bottom = self._box(x1, y1, x2, y2)
        sx = self.canvas.winfo_width() / self.BASE_WIDTH
        sy = self.canvas.winfo_height() / self.BASE_HEIGHT
        r = radius * min(sx, sy)
        points = (
            left + r, top,
            right - r, top,
            right, top,
            right, top + r,
            right, bottom - r,
            right, bottom,
            right - r, bottom,
            left + r, bottom,
            left, bottom,
            left, bottom - r,
            left, top + r,
            left, top,
        )
        self.canvas.create_polygon(
            points,
            smooth=True,
            splinesteps=24,
            fill=fill,
            outline=outline,
            width=max(1.0, width * min(sx, sy)),
        )

    def _text(
        self,
        x: float,
        y: float,
        text: str,
        *,
        size: float,
        fill: str,
        anchor: str = "nw",
        bold: bool = False,
        mono: bool = False,
        width: Optional[float] = None,
        justify: str = "left",
    ) -> None:
        px, py = self._xy(x, y)
        text_width = None
        if width is not None:
            text_width = width * self.canvas.winfo_width() / self.BASE_WIDTH
        self.canvas.create_text(
            px,
            py,
            text=text,
            fill=fill,
            font=self._font(size, bold=bold, mono=mono),
            anchor=anchor,
            width=text_width,
            justify=justify,
        )

    def _line(self, *coordinates: float, **kwargs: object) -> None:
        transformed = []
        for index in range(0, len(coordinates), 2):
            transformed.extend(self._xy(coordinates[index], coordinates[index + 1]))
        scale = min(self.canvas.winfo_width() / self.BASE_WIDTH, self.canvas.winfo_height() / self.BASE_HEIGHT)
        if "width" in kwargs:
            kwargs["width"] = max(1.0, float(kwargs["width"]) * scale)
        self.canvas.create_line(*transformed, **kwargs)

    def _build_background(self, width: int, height: int) -> object:
        if Image is None or ImageDraw is None or ImageTk is None:
            return None
        small_height = max(2, min(height, 720))
        strip = Image.new("RGB", (1, small_height))
        top = (6, 17, 31)
        bottom = (9, 27, 42)
        for y in range(small_height):
            ratio = y / max(1, small_height - 1)
            strip.putpixel((0, y), tuple(int(a + (b - a) * ratio) for a, b in zip(top, bottom)))
        image = strip.resize((width, height))
        overlay = Image.new("RGBA", (width, height), (0, 0, 0, 0))
        draw = ImageDraw.Draw(overlay)
        radius = int(min(width, height) * 0.48)
        draw.ellipse(
            (width // 2 - radius, -radius // 2, width // 2 + radius, radius * 3 // 2),
            fill=(25, 164, 188, 10),
        )
        image = Image.alpha_composite(image.convert("RGBA"), overlay)
        return ImageTk.PhotoImage(image=image)

    def _draw_background(self, width: int, height: int) -> None:
        if self._background_size != (width, height):
            self._background_photo = self._build_background(width, height)
            self._background_size = (width, height)
        if self._background_photo is not None:
            self.canvas.create_image(0, 0, image=self._background_photo, anchor="nw")
        else:
            self.canvas.create_rectangle(0, 0, width, height, fill=self.BG_TOP, outline="")

    def _draw_header(self, snapshot: TelemetrySnapshot) -> None:
        self._text(34, 25, "FIRE RESCUE", size=11, fill=self.CYAN, bold=True)
        self._text(34, 45, "24 GHz LIFE-SIGN MONITOR", size=22, fill=self.WHITE, bold=True)
        self._text(500, 51, "DFROBOT C4001  /  SEN0610", size=10, fill=self.MUTED, mono=True)

        state_colors = {
            "ONLINE": self.GREEN,
            "CONNECTING": self.AMBER,
            "OFFLINE": self.RED,
            "ERROR": self.RED,
        }
        state = snapshot.sensor_state
        color = state_colors.get(state, self.MUTED)
        self._rounded_rectangle(1042, 28, 1246, 73, radius=22, fill="#0A2130", outline=self.CARD_EDGE)
        cx, cy = self._xy(1068, 50)
        pulse = 5.0 + (1.5 * math.sin(snapshot.timestamp * 4.0) if snapshot.online else 0.0)
        sx = self.canvas.winfo_width() / self.BASE_WIDTH
        sy = self.canvas.winfo_height() / self.BASE_HEIGHT
        self.canvas.create_oval(
            cx - pulse * sx,
            cy - pulse * sy,
            cx + pulse * sx,
            cy + pulse * sy,
            fill=color,
            outline="",
        )
        self._text(1084, 43, f"UART  •  {state}", size=11, fill=color, bold=True, mono=True)

    def _draw_presence_card(self, snapshot: TelemetrySnapshot) -> None:
        self._rounded_rectangle(32, 105, 350, 465, fill=self.CARD, outline=self.CARD_EDGE)
        self._text(55, 128, "TARGET STATUS", size=10, fill=self.MUTED, bold=True, mono=True)

        if not snapshot.online:
            main_text, sub_text, color = "센서 연결 대기", "WAITING FOR SENSOR", self.RED
        elif snapshot.presence:
            main_text, sub_text, color = "사람 감지됨", "DETECT", self.GREEN
        else:
            main_text, sub_text, color = "감지 대상 없음", "CLEAR", self.MUTED

        cx, cy = self._xy(79, 190)
        sx = self.canvas.winfo_width() / self.BASE_WIDTH
        sy = self.canvas.winfo_height() / self.BASE_HEIGHT
        outer = 17.0 + (3.0 * (1.0 + math.sin(snapshot.timestamp * 4.5)) if snapshot.presence else 0.0)
        self.canvas.create_oval(
            cx - outer * sx,
            cy - outer * sy,
            cx + outer * sx,
            cy + outer * sy,
            outline=color,
            width=max(1, int(2 * min(sx, sy))),
        )
        self.canvas.create_oval(
            cx - 8 * sx,
            cy - 8 * sy,
            cx + 8 * sx,
            cy + 8 * sy,
            fill=color,
            outline="",
        )
        self._text(111, 169, main_text, size=19, fill=self.WHITE, bold=True)
        self._text(111, 203, sub_text, size=10, fill=color, bold=True, mono=True)

        self._line(55, 244, 327, 244, fill=self.GRID, width=1)
        distance_text = "—"
        if snapshot.presence and snapshot.distance_m is not None:
            distance_text = format_distance_m(snapshot.distance_m, include_unit=False)
        self._text(55, 262, "RADIAL DISTANCE", size=9, fill=self.MUTED, mono=True)
        self._text(53, 286, distance_text, size=42, fill=self.CYAN, bold=True, mono=True)
        if distance_text != "—":
            self._text(251, 309, "m", size=18, fill=self.CYAN, bold=True, mono=True)


    def _draw_radar_card(self, snapshot: TelemetrySnapshot) -> None:
        self._rounded_rectangle(366, 105, 914, 465, fill=self.CARD, outline=self.CARD_EDGE)
        self._text(389, 128, "RADIAL RANGE", size=10, fill=self.MUTED, bold=True, mono=True)
        self._text(891, 128, "0 — 12 m", size=10, fill=self.CYAN, anchor="ne", mono=True)

        center_x, center_y = 640.0, 432.0
        max_radius = 236.0
        sx = self.canvas.winfo_width() / self.BASE_WIDTH
        sy = self.canvas.winfo_height() / self.BASE_HEIGHT
        for distance in (4.0, 8.0, 12.0):
            radius = max_radius * distance / MAX_DISTANCE_M
            box = self._box(center_x - radius, center_y - radius, center_x + radius, center_y + radius)
            self.canvas.create_arc(
                *box,
                start=0,
                extent=180,
                style="arc",
                outline=self.GRID,
                width=max(1, int(1.2 * min(sx, sy))),
            )
            self._text(center_x + 8, center_y - radius - 7, f"{int(distance)}m", size=8, fill=self.MUTED, mono=True)

        self._line(center_x, center_y, center_x, center_y - max_radius, fill="#286176", width=1.4, dash=(3, 5))
        self._line(center_x - max_radius, center_y, center_x + max_radius, center_y, fill=self.GRID, width=1)

        scan_phase = (snapshot.timestamp * 0.17) % 1.0
        scan_y = center_y - max_radius * scan_phase
        self._line(center_x - 20, scan_y, center_x + 20, scan_y, fill="#1A8296", width=1)

        if snapshot.presence and snapshot.distance_m is not None:
            clamped = max(0.0, min(MAX_DISTANCE_M, snapshot.distance_m))
            target_y = center_y - max_radius * clamped / MAX_DISTANCE_M
            tx, ty = self._xy(center_x, target_y)
            pulse = 13.0 + 4.0 * (1.0 + math.sin(snapshot.timestamp * 5.0))
            self.canvas.create_oval(
                tx - pulse * sx,
                ty - pulse * sy,
                tx + pulse * sx,
                ty + pulse * sy,
                outline=self.CYAN,
                width=max(1, int(1.5 * min(sx, sy))),
            )
            self.canvas.create_oval(
                tx - 6 * sx,
                ty - 6 * sy,
                tx + 6 * sx,
                ty + 6 * sy,
                fill=self.GREEN,
                outline=self.WHITE,
                width=max(1, int(min(sx, sy))),
            )
            label_y = max(157.0, target_y - 35.0)
            self._rounded_rectangle(center_x + 18, label_y, center_x + 126, label_y + 30, radius=8, fill="#0B2B36", outline="#1D6C78")
            self._text(center_x + 30, label_y + 7, format_distance_m(snapshot.distance_m), size=10, fill=self.CYAN, bold=True, mono=True)

        self._text(
            389,
            451,
            "거리축만 표시  ·  각도 데이터 미제공",
            size=8,
            fill=self.MUTED,
            anchor="sw",
        )

    def _draw_mobility_card(self, snapshot: TelemetrySnapshot) -> None:
        self._rounded_rectangle(930, 105, 1248, 465, fill=self.CARD, outline=self.CARD_EDGE)
        presentation = MOBILITY_PRESENTATIONS[snapshot.mobility_state]
        self._text(953, 128, "MOBILITY ASSESSMENT", size=10, fill=self.MUTED, bold=True, mono=True)

        self._rounded_rectangle(953, 164, 1225, 205, radius=20, fill="#091925", outline=presentation.color)
        self._text(1089, 184, presentation.subtitle, size=10, fill=presentation.color, anchor="center", bold=True, mono=True)

        self._text(953, 231, presentation.title, size=20, fill=self.WHITE, bold=True)
        self._text(953, 271, presentation.detail, size=11, fill=self.MUTED, width=265)

        self._line(953, 330, 1225, 330, fill=self.GRID, width=1)
        self._text(953, 349, "연속 무동작 시간", size=9, fill=self.MUTED)
        duration_color = presentation.color if snapshot.presence else self.MUTED
        self._text(
            1225,
            344,
            format_duration(snapshot.still_duration_sec),
            size=18,
            fill=duration_color,
            anchor="ne",
            bold=True,
            mono=True,
        )

        self._rounded_rectangle(953, 392, 1225, 443, radius=9, fill="#111E29", outline="#293B49")
        self._text(
            967,
            403,
            "※ 레이더 기반 추정이며 의식·자세를 판정하지 않습니다.",
            size=8,
            fill="#9AAEBA",
            width=244,
        )

    def _draw_chart(
        self,
        *,
        x1: float,
        x2: float,
        title: str,
        current: str,
        samples: Sequence[HistoryPoint],
        snapshot: TelemetrySnapshot,
        color: str,
        value_min: float,
        value_max: float,
        max_label: str,
        min_label: str,
    ) -> None:
        y1, y2 = 482.0, 697.0
        self._rounded_rectangle(x1, y1, x2, y2, fill=self.CARD, outline=self.CARD_EDGE)
        self._text(x1 + 20, y1 + 17, title, size=9, fill=self.MUTED, bold=True, mono=True)
        self._text(x2 - 20, y1 + 15, current, size=12, fill=color, anchor="ne", bold=True, mono=True)

        graph_x = x1 + 46
        graph_y = y1 + 59
        graph_w = x2 - x1 - 65
        graph_h = 113.0
        self._rounded_rectangle(graph_x, graph_y, graph_x + graph_w, graph_y + graph_h, radius=5, fill="#081824", outline=self.GRID)
        for ratio in (0.25, 0.5, 0.75):
            line_y = graph_y + graph_h * ratio
            self._line(graph_x, line_y, graph_x + graph_w, line_y, fill="#123342", width=1, dash=(2, 5))
        self._text(graph_x - 7, graph_y - 3, max_label, size=7, fill=self.MUTED, anchor="ne", mono=True)
        self._text(graph_x - 7, graph_y + graph_h - 7, min_label, size=7, fill=self.MUTED, anchor="ne", mono=True)
        self._text(graph_x, graph_y + graph_h + 9, "−30s", size=7, fill=self.MUTED, mono=True)
        self._text(graph_x + graph_w, graph_y + graph_h + 9, "NOW", size=7, fill=self.MUTED, anchor="ne", mono=True)

        points = chart_coordinates(
            samples,
            now=snapshot.timestamp,
            window_sec=HISTORY_WINDOW_SEC,
            value_min=value_min,
            value_max=value_max,
            x=graph_x,
            y=graph_y,
            width=graph_w,
            height=graph_h,
        )
        if len(points) >= 2:
            transformed = []
            for point_x, point_y in points:
                transformed.extend(self._xy(point_x, point_y))
            self.canvas.create_line(
                *transformed,
                fill=color,
                width=max(1, int(2 * min(self.canvas.winfo_width() / self.BASE_WIDTH, self.canvas.winfo_height() / self.BASE_HEIGHT))),
                smooth=False,
                splinesteps=12,
            )
        elif len(points) == 1:
            point_x, point_y = self._xy(*points[0])
            scale = min(self.canvas.winfo_width() / self.BASE_WIDTH, self.canvas.winfo_height() / self.BASE_HEIGHT)
            radius = 3 * scale
            self.canvas.create_oval(point_x - radius, point_y - radius, point_x + radius, point_y + radius, fill=color, outline="")
        else:
            self._text(graph_x + graph_w / 2, graph_y + graph_h / 2, "NO DATA", size=9, fill="#456172", anchor="center", mono=True)

    def _draw_charts(self, snapshot: TelemetrySnapshot) -> None:
        activity = max(0.0, min(100.0, snapshot.motion_activity_percent))
        color = self.GREEN if snapshot.mobility_state is MobilityState.MOVING else self.CYAN
        self._draw_chart(
            x1=32,
            x2=1248,
            title="MOVEMENT ACTIVITY  ·  LAST 30 SEC",
            current=f"{activity:.0f}%",
            samples=snapshot.motion_history,
            snapshot=snapshot,
            color=color,
            value_min=0.0,
            value_max=100.0,
            max_label="100%",
            min_label="0%",
        )

    def _render(self, snapshot: TelemetrySnapshot) -> None:
        width = max(2, self.canvas.winfo_width())
        height = max(2, self.canvas.winfo_height())
        self.canvas.delete("all")
        self._draw_background(width, height)
        self._draw_header(snapshot)
        self._draw_presence_card(snapshot)
        self._draw_radar_card(snapshot)
        self._draw_mobility_card(snapshot)
        self._draw_charts(snapshot)


def main(args: Optional[Sequence[str]] = None) -> None:
    """Start ROS in a worker thread and keep Tk on the process main thread."""

    if not ROS_AVAILABLE:
        raise RuntimeError("ROS 2 Python (rclpy/std_msgs) is not available; source the ROS environment first")
    if tk is None:
        raise RuntimeError("Tkinter is required to run the mmWave visualizer")

    rclpy.init(args=list(args) if args is not None else None)
    store = TelemetryStore()
    node = MmWaveGuiNode(store)
    stop_event = threading.Event()

    def spin_ros() -> None:
        while not stop_event.is_set() and rclpy.ok():
            try:
                rclpy.spin_once(node, timeout_sec=0.1)
            except Exception as exc:  # Keep the UI alive long enough to show OFFLINE.
                if stop_event.is_set() or not rclpy.ok():
                    break
                if not stop_event.is_set():
                    node.get_logger().error(f"ROS spin error: {exc}")
                    time.sleep(0.1)

    spin_thread = threading.Thread(target=spin_ros, name="mmwave-ros-spin", daemon=True)
    spin_thread.start()

    try:
        root = tk.Tk()
        dashboard = MmWaveDashboard(
            root,
            store,
            on_close=stop_event.set,
            should_close=lambda: not rclpy.ok(),
        )
        try:
            dashboard.run()
        except KeyboardInterrupt:
            dashboard.close()
    finally:
        stop_event.set()
        spin_thread.join(timeout=2.0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
