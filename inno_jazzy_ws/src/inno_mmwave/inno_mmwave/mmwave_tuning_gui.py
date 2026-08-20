#!/usr/bin/env python3
"""Standalone UART tuning GUI for the DFRobot C4001/SEN0610.

The normal ``c4001_node`` must be stopped while this program owns the UART.
Sensor changes are deliberately volatile: no ``saveConfig`` command is sent.
"""

from __future__ import annotations

import argparse
from collections import deque
import csv
from dataclasses import dataclass, replace
from datetime import datetime
import math
from pathlib import Path
from queue import Empty, Queue
import threading
import time
from typing import Callable, Deque, Dict, Optional, Tuple

try:
    import tkinter as tk
    from tkinter import ttk
except ImportError:  # pragma: no cover - target dependency
    tk = None  # type: ignore[assignment]
    ttk = None  # type: ignore[assignment]

try:
    import serial
except ImportError:  # pragma: no cover - target dependency
    serial = None  # type: ignore[assignment]

from .c4001_protocol import C4001Measurement, C4001StreamParser, encode_command
from .mmwave_processing import (
    DistanceProcessor,
    FilterType,
    HumanCandidateDetector,
    HumanTuningSettings,
    ProcessedRange,
    ProcessingSettings,
)


MIN_SENSOR_RANGE_CM = 30
MIN_SENSOR_MAX_RANGE_CM = 240
MAX_SENSOR_RANGE_CM = 2000
MAX_THRESHOLD = 65535
HISTORY_WINDOW_SEC = 20.0
CSV_FIELDS = (
    "timestamp",
    "target_number",
    "raw_range",
    "calibrated_range",
    "filtered_range",
    "speed",
    "energy",
    "sensor_min_range",
    "sensor_max_range",
    "sensor_threshold",
    "fretting",
    "scale",
    "offset",
    "filter_type",
    "filter_size",
    "ema_alpha",
    "outlier_threshold",
    "human_candidate",
)


@dataclass(frozen=True)
class SensorSettings:
    min_range_cm: int = 120
    max_range_cm: int = 1200
    threshold: int = 10
    fretting_enabled: bool = True

    def __post_init__(self) -> None:
        if isinstance(self.min_range_cm, bool) or not (
            MIN_SENSOR_RANGE_CM <= self.min_range_cm < self.max_range_cm
        ):
            raise ValueError(
                f"Min Range must be {MIN_SENSOR_RANGE_CM} cm or greater and below Max Range"
            )
        if isinstance(self.max_range_cm, bool) or not (
            MIN_SENSOR_MAX_RANGE_CM <= self.max_range_cm <= MAX_SENSOR_RANGE_CM
        ):
            raise ValueError(
                f"Max Range must be {MIN_SENSOR_MAX_RANGE_CM}..{MAX_SENSOR_RANGE_CM} cm"
            )
        if isinstance(self.threshold, bool) or not 0 <= self.threshold <= MAX_THRESHOLD:
            raise ValueError(f"Detection Threshold must be 0..{MAX_THRESHOLD}")
        if not isinstance(self.fretting_enabled, bool):
            raise ValueError("Fretting Detection must be ON or OFF")

    def volatile_commands(self) -> Tuple[str, ...]:
        """Official UART settings without saveConfig (RAM/session only)."""

        return (
            "sensorStop",
            "setRunApp 1",
            f"setRange {self.min_range_cm / 100.0:g} {self.max_range_cm / 100.0:g}",
            f"setThrFactor {self.threshold}",
            f"setMicroMotion {1 if self.fretting_enabled else 0}",
            "sensorStart",
        )


@dataclass(frozen=True)
class TuningSnapshot:
    monotonic_time: float
    wall_time: str
    connection_state: str
    target_number: int
    raw_range_m: Optional[float]
    calibrated_range_m: Optional[float]
    filtered_range_m: Optional[float]
    speed_mps: Optional[float]
    energy: Optional[int]
    human_candidate: bool
    sample_state: str
    sensor_settings: SensorSettings
    processing_settings: ProcessingSettings
    human_settings: HumanTuningSettings
    distance_history: Tuple[Tuple[float, float, float, float], ...]
    energy_history: Tuple[Tuple[float, float], ...]


class TuningStore:
    """Thread-safe processor and snapshot store shared by UART and Tk."""

    def __init__(self, sensor_settings: SensorSettings) -> None:
        self._lock = threading.RLock()
        self.sensor_settings = sensor_settings
        self.processor = DistanceProcessor()
        self.detector = HumanCandidateDetector()
        self.connection_state = "CONNECTING"
        self.target_number = 0
        self.raw_range_m: Optional[float] = None
        self.calibrated_range_m: Optional[float] = None
        self.filtered_range_m: Optional[float] = None
        self.speed_mps: Optional[float] = None
        self.energy: Optional[int] = None
        self.human_candidate = False
        self.sample_state = "NO TARGET"
        self._distance_history: Deque[Tuple[float, float, float, float]] = deque()
        self._energy_history: Deque[Tuple[float, float]] = deque()

    def set_connection_state(self, state: str) -> None:
        with self._lock:
            self.connection_state = state

    def set_sensor_settings(self, settings: SensorSettings) -> None:
        with self._lock:
            self.sensor_settings = settings

    def apply_processing(self, settings: ProcessingSettings) -> None:
        with self._lock:
            self.processor.apply_settings(settings)

    def apply_human(self, settings: HumanTuningSettings) -> None:
        with self._lock:
            self.detector.apply_settings(settings)
            self.human_candidate = False

    def reset_software(self) -> None:
        with self._lock:
            self.processor.reset()
            self.detector.apply_settings(HumanTuningSettings())
            self.human_candidate = False

    def clear_filter(self) -> None:
        with self._lock:
            self.processor.clear_filter_buffer()

    def update(self, measurement: C4001Measurement) -> TuningSnapshot:
        now = time.monotonic()
        with self._lock:
            valid_target = measurement.detected and measurement.distance_m is not None
            processed: ProcessedRange = self.processor.process(
                measurement.distance_m, target_valid=valid_target
            )
            self.target_number = measurement.target_count
            self.raw_range_m = processed.raw_range_m
            self.calibrated_range_m = processed.calibrated_range_m
            self.filtered_range_m = processed.filtered_range_m
            self.speed_mps = measurement.speed_mps if valid_target else None
            self.energy = measurement.energy if valid_target else None
            self.sample_state = processed.rejection_reason or "TARGET"
            self.human_candidate = self.detector.update(
                target_count=self.target_number,
                filtered_range_m=self.filtered_range_m,
                energy=self.energy,
            )
            if processed.accepted:
                assert processed.raw_range_m is not None
                assert processed.calibrated_range_m is not None
                assert processed.filtered_range_m is not None
                self._distance_history.append(
                    (
                        now,
                        processed.raw_range_m,
                        processed.calibrated_range_m,
                        processed.filtered_range_m,
                    )
                )
                if self.energy is not None:
                    self._energy_history.append((now, float(self.energy)))
            self._prune(now)
            return self._snapshot_locked(now)

    def _prune(self, now: float) -> None:
        cutoff = now - HISTORY_WINDOW_SEC
        while self._distance_history and self._distance_history[0][0] < cutoff:
            self._distance_history.popleft()
        while self._energy_history and self._energy_history[0][0] < cutoff:
            self._energy_history.popleft()

    def _snapshot_locked(self, now: float) -> TuningSnapshot:
        return TuningSnapshot(
            monotonic_time=now,
            wall_time=datetime.now().astimezone().isoformat(timespec="milliseconds"),
            connection_state=self.connection_state,
            target_number=self.target_number,
            raw_range_m=self.raw_range_m,
            calibrated_range_m=self.calibrated_range_m,
            filtered_range_m=self.filtered_range_m,
            speed_mps=self.speed_mps,
            energy=self.energy,
            human_candidate=self.human_candidate,
            sample_state=self.sample_state,
            sensor_settings=self.sensor_settings,
            processing_settings=self.processor.settings,
            human_settings=self.detector.settings,
            distance_history=tuple(self._distance_history),
            energy_history=tuple(self._energy_history),
        )

    def snapshot(self) -> TuningSnapshot:
        now = time.monotonic()
        with self._lock:
            self._prune(now)
            return self._snapshot_locked(now)


class CsvRecorder:
    """Queue-backed CSV writer so disk I/O never runs in the UART/Tk loops."""

    def __init__(self, log_directory: Path) -> None:
        self.log_directory = log_directory
        self._queue: Queue[Optional[TuningSnapshot]] = Queue(maxsize=2000)
        self._thread: Optional[threading.Thread] = None
        self._recording = threading.Event()
        self.current_path: Optional[Path] = None

    @property
    def recording(self) -> bool:
        return self._recording.is_set()

    def start(self) -> Path:
        if self.recording:
            assert self.current_path is not None
            return self.current_path
        self.log_directory.mkdir(parents=True, exist_ok=True)
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.current_path = self.log_directory / f"mmwave_tuning_{stamp}.csv"
        self._recording.set()
        self._thread = threading.Thread(
            target=self._writer, name="mmwave-csv-writer", daemon=True
        )
        self._thread.start()
        return self.current_path

    def submit(self, snapshot: TuningSnapshot) -> None:
        if not self.recording:
            return
        try:
            self._queue.put_nowait(snapshot)
        except Exception:
            # Losing a row is safer than blocking sensor reads if storage stalls.
            pass

    def stop(self) -> None:
        if not self.recording:
            return
        self._recording.clear()
        self._queue.put(None)
        if self._thread is not None:
            self._thread.join(timeout=3.0)
        self._thread = None

    @staticmethod
    def _row(snapshot: TuningSnapshot) -> Dict[str, object]:
        sensor = snapshot.sensor_settings
        software = snapshot.processing_settings
        return {
            "timestamp": snapshot.wall_time,
            "target_number": snapshot.target_number,
            "raw_range": snapshot.raw_range_m,
            "calibrated_range": snapshot.calibrated_range_m,
            "filtered_range": snapshot.filtered_range_m,
            "speed": snapshot.speed_mps,
            "energy": snapshot.energy,
            "sensor_min_range": sensor.min_range_cm,
            "sensor_max_range": sensor.max_range_cm,
            "sensor_threshold": sensor.threshold,
            "fretting": "ON" if sensor.fretting_enabled else "OFF",
            "scale": software.scale,
            "offset": software.offset_m,
            "filter_type": software.filter_type.value,
            "filter_size": software.filter_size,
            "ema_alpha": software.ema_alpha,
            "outlier_threshold": software.outlier_threshold_m,
            "human_candidate": snapshot.human_candidate,
        }

    def _writer(self) -> None:
        assert self.current_path is not None
        try:
            with self.current_path.open("w", newline="", encoding="utf-8") as handle:
                writer = csv.DictWriter(handle, fieldnames=CSV_FIELDS)
                writer.writeheader()
                while True:
                    item = self._queue.get()
                    if item is None:
                        break
                    writer.writerow(self._row(item))
                handle.flush()
        except OSError as error:
            print(f"mmwave_tuning_gui CSV error: {error}", flush=True)
        finally:
            self._recording.clear()


class C4001UartWorker:
    """Single-owner UART worker; reads and writes share one serial lock."""

    def __init__(
        self,
        *,
        port: str,
        baud_rate: int,
        store: TuningStore,
        recorder: CsvRecorder,
        status_callback: Callable[[str], None],
    ) -> None:
        self.port = port
        self.baud_rate = baud_rate
        self.store = store
        self.recorder = recorder
        self.status_callback = status_callback
        self._serial_lock = threading.Lock()
        self._commands: Queue[Tuple[SensorSettings, str]] = Queue()
        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self._port_handle = None
        self._parser = C4001StreamParser()

    def start(self) -> None:
        self._thread = threading.Thread(
            target=self._run, name="c4001-uart-worker", daemon=True
        )
        self._thread.start()

    def request_settings(self, settings: SensorSettings, description: str) -> None:
        self._commands.put((settings, description))

    def close(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=3.0)
        self._close_port()

    def _notify(self, text: str) -> None:
        print(f"mmwave_tuning_gui: {text}", flush=True)
        self.status_callback(text)

    def _open_port(self) -> bool:
        if serial is None:
            self.store.set_connection_state("SENSOR ERROR")
            self._notify("SENSOR ERROR: python3-serial is not installed")
            return False
        try:
            self._port_handle = serial.Serial(
                port=self.port,
                baudrate=self.baud_rate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=0.05,
                write_timeout=0.5,
                exclusive=True,
            )
            self._port_handle.reset_input_buffer()
            self._parser.reset()
            self.store.set_connection_state("CONNECTED")
            self._notify(f"CONNECTED: {self.port} at {self.baud_rate} baud")
            return True
        except (serial.SerialException, OSError, ValueError) as error:
            self._port_handle = None
            self.store.set_connection_state("SENSOR ERROR")
            self._notify(f"SENSOR ERROR: cannot open {self.port}: {error}")
            return False

    def _close_port(self) -> None:
        handle = self._port_handle
        self._port_handle = None
        if handle is not None:
            try:
                handle.close()
            except Exception:
                pass

    def _apply(self, settings: SensorSettings, description: str) -> None:
        assert self._port_handle is not None
        self.store.set_connection_state("APPLYING")
        try:
            with self._serial_lock:
                for command in settings.volatile_commands():
                    if command == "sensorStart":
                        self._port_handle.reset_input_buffer()
                        self._parser.reset()
                    self._port_handle.write(encode_command(command))
                    self._port_handle.flush()
                    if command == "sensorStop":
                        time.sleep(1.0)
                    else:
                        time.sleep(0.15)
            self.store.set_sensor_settings(settings)
            self.store.set_connection_state("CONNECTED")
            self._notify(f"Applied: {description} (volatile; not saved to sensor flash)")
        except (serial.SerialException, OSError) as error:
            self.store.set_connection_state("SENSOR ERROR")
            self._notify(f"SENSOR ERROR while applying settings: {error}")
            self._close_port()

    def _read(self) -> None:
        assert self._port_handle is not None
        with self._serial_lock:
            data = self._port_handle.read(512)
        if not data:
            return
        for measurement in self._parser.feed(data):
            snapshot = self.store.update(measurement)
            self.recorder.submit(snapshot)

    def _run(self) -> None:
        next_reconnect = 0.0
        while not self._stop.is_set():
            if self._port_handle is None:
                if time.monotonic() < next_reconnect:
                    self._stop.wait(0.2)
                    continue
                if not self._open_port():
                    next_reconnect = time.monotonic() + 2.0
                    self._stop.wait(0.2)
                    continue
                self._commands.put((self.store.sensor_settings, "startup settings"))
            try:
                try:
                    settings, description = self._commands.get_nowait()
                except Empty:
                    settings = None
                    description = ""
                if settings is not None:
                    self._apply(settings, description)
                if self._port_handle is not None:
                    self._read()
            except (serial.SerialException, OSError) as error:
                self.store.set_connection_state("SENSOR ERROR")
                self._notify(f"SENSOR ERROR during read: {error}")
                self._close_port()
                next_reconnect = time.monotonic() + 2.0
        self._close_port()


class RealtimeGraph(tk.Canvas if tk is not None else object):
    """Small dependency-free Tk graph for distance and energy histories."""

    COLORS = ("#eb5757", "#2d9cdb", "#27ae60")

    def __init__(self, master: object) -> None:
        super().__init__(master, height=270, bg="#111827", highlightthickness=0)

    def draw(self, snapshot: TuningSnapshot) -> None:
        self.delete("all")
        width = max(400, self.winfo_width())
        height = max(220, self.winfo_height())
        left, right, top, distance_bottom = 55, width - 18, 20, int(height * 0.64)
        energy_top, bottom = distance_bottom + 35, height - 25
        self.create_text(left, 5, text="Distance [m]", fill="#e5e7eb", anchor="nw")
        self.create_text(left, energy_top - 18, text="Energy", fill="#e5e7eb", anchor="nw")
        now = snapshot.monotonic_time
        distances = snapshot.distance_history
        values = [value for row in distances for value in row[1:]]
        y_max = max(2.0, max(values, default=2.0) * 1.10)
        energy_max = max(100.0, max((v for _, v in snapshot.energy_history), default=100.0) * 1.10)

        for y1, y2 in ((top, distance_bottom), (energy_top, bottom)):
            self.create_rectangle(left, y1, right, y2, outline="#374151")
            for index in range(1, 4):
                y = y1 + (y2 - y1) * index / 4
                self.create_line(left, y, right, y, fill="#263244")

        def point(timestamp: float, value: float, y1: float, y2: float, maximum: float):
            x = left + (timestamp - (now - HISTORY_WINDOW_SEC)) / HISTORY_WINDOW_SEC * (right - left)
            y = y2 - min(max(value, 0.0), maximum) / maximum * (y2 - y1)
            return x, y

        names = ("RAW", "CALIBRATED", "FILTERED")
        for series_index, (name, color) in enumerate(zip(names, self.COLORS), start=1):
            coordinates = []
            for row in distances:
                coordinates.extend(point(row[0], row[series_index], top, distance_bottom, y_max))
            if len(coordinates) >= 4:
                self.create_line(*coordinates, fill=color, width=2)
            self.create_text(
                left + (series_index - 1) * 130,
                distance_bottom + 7,
                text=name,
                fill=color,
                anchor="nw",
            )
        energy_coordinates = []
        for timestamp, value in snapshot.energy_history:
            energy_coordinates.extend(point(timestamp, value, energy_top, bottom, energy_max))
        if len(energy_coordinates) >= 4:
            self.create_line(*energy_coordinates, fill="#f2c94c", width=2)
        self.create_text(5, top, text=f"{y_max:.1f}", fill="#9ca3af", anchor="nw")
        self.create_text(5, distance_bottom - 12, text="0", fill="#9ca3af", anchor="nw")
        self.create_text(5, energy_top, text=f"{energy_max:.0f}", fill="#9ca3af", anchor="nw")
        self.create_text(right, bottom + 3, text="now", fill="#9ca3af", anchor="ne")


class TuningGui:
    def __init__(
        self,
        root: "tk.Tk",
        store: TuningStore,
        worker: C4001UartWorker,
        recorder: CsvRecorder,
        log_directory: Path,
    ) -> None:
        self.root = root
        self.store = store
        self.worker = worker
        self.recorder = recorder
        self.log_directory = log_directory
        self._status_queue: Queue[str] = Queue()
        self._closing = False
        root.title("C4001 mmWave Tuning Tool")
        root.geometry("1280x900")
        root.minsize(1040, 760)
        root.protocol("WM_DELETE_WINDOW", self.close)

        self.status_var = tk.StringVar(value="CONNECTING")
        self.record_var = tk.StringVar(value=f"CSV directory: {log_directory}")
        self.live_vars = {name: tk.StringVar(value="—") for name in (
            "target", "raw", "calibrated", "filtered", "speed", "energy", "sample"
        )}
        self.current_var = tk.StringVar(value="")
        self.human_var = tk.StringVar(value="NO HUMAN")
        self.entries: Dict[str, tk.StringVar] = {}

        outer = ttk.Frame(root, padding=10)
        outer.pack(fill="both", expand=True)
        ttk.Label(outer, text="C4001 mmWave Tuning Tool", font=("TkDefaultFont", 18, "bold")).pack()
        ttk.Label(
            outer,
            text="Experimental heuristic only — HUMAN CANDIDATE is not an AI classification. Stop c4001_node while this tool owns UART.",
        ).pack(pady=(2, 8))
        upper = ttk.Frame(outer)
        upper.pack(fill="x")
        upper.columnconfigure((0, 1, 2, 3), weight=1, uniform="panels")

        sensor_frame = ttk.LabelFrame(upper, text="Sensor Settings (volatile)", padding=8)
        sensor_frame.grid(row=0, column=0, sticky="nsew", padx=3)
        self._entry(sensor_frame, "Min Range [cm]", "min_range", "120", self.apply_sensor)
        self._entry(sensor_frame, "Max Range [cm]", "max_range", "1200", self.apply_sensor)
        self._entry(sensor_frame, "Detection Threshold", "threshold", "10", self.apply_sensor)
        self.entries["fretting"] = tk.StringVar(value="ON")
        ttk.Label(sensor_frame, text="Fretting Detection").grid(row=3, column=0, sticky="w", pady=2)
        fretting = ttk.Combobox(sensor_frame, textvariable=self.entries["fretting"], values=("ON", "OFF"), state="readonly", width=13)
        fretting.grid(row=3, column=1, sticky="ew", pady=2)
        fretting.bind("<<ComboboxSelected>>", self.apply_sensor)
        ttk.Button(sensor_frame, text="Apply Sensor Settings", command=self.apply_sensor).grid(row=4, column=0, columnspan=2, sticky="ew", pady=8)

        live_frame = ttk.LabelFrame(upper, text="Live Sensor Data", padding=8)
        live_frame.grid(row=0, column=1, sticky="nsew", padx=3)
        for row, (label, key) in enumerate((
            ("Target Number", "target"), ("Raw Range", "raw"),
            ("Calibrated Range", "calibrated"), ("Filtered Range", "filtered"),
            ("Speed", "speed"), ("Energy", "energy"), ("Target State", "sample"),
        )):
            ttk.Label(live_frame, text=label).grid(row=row, column=0, sticky="w", pady=2)
            ttk.Label(live_frame, textvariable=self.live_vars[key], font=("TkFixedFont", 10, "bold")).grid(row=row, column=1, sticky="e", padx=8)

        software_frame = ttk.LabelFrame(upper, text="Distance Calibration / Filter", padding=8)
        software_frame.grid(row=0, column=2, sticky="nsew", padx=3)
        self._entry(software_frame, "Scale", "scale", "1.0000", self.apply_software, row=0)
        self._entry(software_frame, "Offset [m]", "offset", "0.0000", self.apply_software, row=1)
        self.entries["filter_type"] = tk.StringVar(value=FilterType.NONE.value)
        ttk.Label(software_frame, text="Filter Type").grid(row=2, column=0, sticky="w", pady=2)
        filter_box = ttk.Combobox(software_frame, textvariable=self.entries["filter_type"], values=tuple(item.value for item in FilterType), state="readonly", width=14)
        filter_box.grid(row=2, column=1, sticky="ew", pady=2)
        filter_box.bind("<<ComboboxSelected>>", self.apply_software)
        self._entry(software_frame, "Filter Size", "filter_size", "5", self.apply_software, row=3)
        self._entry(software_frame, "EMA Alpha", "ema_alpha", "0.30", self.apply_software, row=4)
        self._entry(software_frame, "Outlier Threshold [m]", "outlier", "0.00", self.apply_software, row=5)
        buttons = ttk.Frame(software_frame)
        buttons.grid(row=6, column=0, columnspan=2, sticky="ew", pady=6)
        ttk.Button(buttons, text="Reset Software Tuning", command=self.reset_software).pack(fill="x")
        ttk.Button(buttons, text="Clear Filter Buffer", command=self.clear_filter).pack(fill="x", pady=(4, 0))

        human_frame = ttk.LabelFrame(upper, text="Human Detection Tuning", padding=8)
        human_frame.grid(row=0, column=3, sticky="nsew", padx=3)
        self._entry(human_frame, "Human Range Min [m]", "human_min", "1.2", self.apply_human, row=0)
        self._entry(human_frame, "Human Range Max [m]", "human_max", "1.8", self.apply_human, row=1)
        self._entry(human_frame, "Energy Threshold", "energy_threshold", "0", self.apply_human, row=2)
        self._entry(human_frame, "Confirm Frames", "confirm_frames", "3", self.apply_human, row=3)
        self._entry(human_frame, "Clear Frames", "clear_frames", "3", self.apply_human, row=4)
        self.human_label = tk.Label(human_frame, textvariable=self.human_var, font=("TkDefaultFont", 15, "bold"), fg="white", bg="#6b7280", pady=12)
        self.human_label.grid(row=5, column=0, columnspan=2, sticky="ew", pady=8)

        middle = ttk.Frame(outer)
        middle.pack(fill="x", pady=8)
        current = ttk.LabelFrame(middle, text="CURRENT SETTINGS", padding=8)
        current.pack(side="left", fill="both", expand=True, padx=(0, 4))
        ttk.Label(current, textvariable=self.current_var, font=("TkFixedFont", 9), justify="left").pack(anchor="w")
        recording = ttk.LabelFrame(middle, text="CSV Recording", padding=8)
        recording.pack(side="right", fill="both", expand=True, padx=(4, 0))
        ttk.Label(recording, textvariable=self.record_var, wraplength=540).pack(anchor="w")
        row = ttk.Frame(recording)
        row.pack(fill="x", pady=5)
        ttk.Button(row, text="Start Recording", command=self.start_recording).pack(side="left", expand=True, fill="x")
        ttk.Button(row, text="Stop Recording", command=self.stop_recording).pack(side="left", expand=True, fill="x", padx=(5, 0))

        graph_frame = ttk.LabelFrame(outer, text="Realtime Graph — RAW / CALIBRATED / FILTERED RANGE / ENERGY", padding=4)
        graph_frame.pack(fill="both", expand=True)
        self.graph = RealtimeGraph(graph_frame)
        self.graph.pack(fill="both", expand=True)
        status = tk.Label(outer, textvariable=self.status_var, anchor="w", fg="white", bg="#374151", padx=8, pady=5)
        status.pack(fill="x", pady=(7, 0))
        self._tick()

    def post_status(self, text: str) -> None:
        self._status_queue.put(text)

    def _entry(self, parent, label: str, key: str, default: str, callback, row: Optional[int] = None) -> None:
        if row is None:
            row = len([key for key in self.entries if key != "fretting" and key != "filter_type"])
        variable = tk.StringVar(value=default)
        self.entries[key] = variable
        ttk.Label(parent, text=label).grid(row=row, column=0, sticky="w", pady=2)
        entry = ttk.Entry(parent, textvariable=variable, width=14)
        entry.grid(row=row, column=1, sticky="ew", pady=2)
        entry.bind("<Return>", callback)

    def _error(self, message: str) -> None:
        self.status_var.set(f"INVALID PARAMETER: {message}")
        print(f"mmwave_tuning_gui: INVALID PARAMETER: {message}", flush=True)

    def apply_sensor(self, _event=None) -> None:
        try:
            settings = SensorSettings(
                min_range_cm=int(self.entries["min_range"].get()),
                max_range_cm=int(self.entries["max_range"].get()),
                threshold=int(self.entries["threshold"].get()),
                fretting_enabled=self.entries["fretting"].get() == "ON",
            )
        except (ValueError, TypeError) as error:
            self._error(str(error) or "Invalid Sensor Settings")
            return
        self.worker.request_settings(settings, self._sensor_description(settings))
        self.status_var.set("Applying sensor settings…")

    @staticmethod
    def _sensor_description(settings: SensorSettings) -> str:
        return (
            f"Min={settings.min_range_cm} cm, Max={settings.max_range_cm} cm, "
            f"Threshold={settings.threshold}, Fretting={'ON' if settings.fretting_enabled else 'OFF'}"
        )

    def apply_software(self, _event=None) -> None:
        try:
            settings = ProcessingSettings(
                scale=float(self.entries["scale"].get()),
                offset_m=float(self.entries["offset"].get()),
                filter_type=FilterType(self.entries["filter_type"].get()),
                filter_size=int(self.entries["filter_size"].get()),
                ema_alpha=float(self.entries["ema_alpha"].get()),
                outlier_threshold_m=float(self.entries["outlier"].get()),
            )
            self.store.apply_processing(settings)
        except (ValueError, TypeError) as error:
            self._error(str(error))
            return
        self.status_var.set("Applied software tuning immediately")

    def apply_human(self, _event=None) -> None:
        try:
            settings = HumanTuningSettings(
                range_min_m=float(self.entries["human_min"].get()),
                range_max_m=float(self.entries["human_max"].get()),
                energy_threshold=int(self.entries["energy_threshold"].get()),
                confirm_frames=int(self.entries["confirm_frames"].get()),
                clear_frames=int(self.entries["clear_frames"].get()),
            )
            self.store.apply_human(settings)
        except (ValueError, TypeError) as error:
            self._error(str(error))
            return
        self.status_var.set("Applied human-candidate heuristic immediately")

    def reset_software(self) -> None:
        self.store.reset_software()
        defaults = ProcessingSettings()
        human = HumanTuningSettings()
        for key, value in (
            ("scale", f"{defaults.scale:.4f}"), ("offset", f"{defaults.offset_m:.4f}"),
            ("filter_type", defaults.filter_type.value), ("filter_size", str(defaults.filter_size)),
            ("ema_alpha", f"{defaults.ema_alpha:.2f}"), ("outlier", "0.00"),
            ("human_min", str(human.range_min_m)), ("human_max", str(human.range_max_m)),
            ("energy_threshold", str(human.energy_threshold)),
            ("confirm_frames", str(human.confirm_frames)), ("clear_frames", str(human.clear_frames)),
        ):
            self.entries[key].set(value)
        self.status_var.set("Software tuning reset; sensor settings unchanged")

    def clear_filter(self) -> None:
        self.store.clear_filter()
        self.status_var.set("Filter buffer cleared")

    def start_recording(self) -> None:
        try:
            path = self.recorder.start()
        except OSError as error:
            self.status_var.set(f"CSV ERROR: {error}")
            return
        self.record_var.set(f"Recording: {path}")
        self.status_var.set("CSV recording started")

    def stop_recording(self) -> None:
        path = self.recorder.current_path
        self.recorder.stop()
        self.record_var.set(f"Saved: {path}" if path else f"CSV directory: {self.log_directory}")
        self.status_var.set("CSV recording stopped")

    @staticmethod
    def _distance(value: Optional[float]) -> str:
        return "—" if value is None else f"{value:.3f} m"

    def _tick(self) -> None:
        if self._closing:
            return
        try:
            while True:
                self.status_var.set(self._status_queue.get_nowait())
        except Empty:
            pass
        snapshot = self.store.snapshot()
        self.live_vars["target"].set(str(snapshot.target_number))
        self.live_vars["raw"].set(self._distance(snapshot.raw_range_m))
        self.live_vars["calibrated"].set(self._distance(snapshot.calibrated_range_m))
        self.live_vars["filtered"].set(self._distance(snapshot.filtered_range_m))
        self.live_vars["speed"].set("—" if snapshot.speed_mps is None else f"{snapshot.speed_mps:.3f} m/s")
        self.live_vars["energy"].set("—" if snapshot.energy is None else str(snapshot.energy))
        self.live_vars["sample"].set(snapshot.sample_state)
        self.human_var.set("HUMAN CANDIDATE" if snapshot.human_candidate else "NO HUMAN")
        self.human_label.configure(bg="#15803d" if snapshot.human_candidate else "#6b7280")
        sensor, software = snapshot.sensor_settings, snapshot.processing_settings
        self.current_var.set(
            "Sensor\n"
            f"  Min / Max      : {sensor.min_range_cm} / {sensor.max_range_cm} cm\n"
            f"  Threshold      : {sensor.threshold}\n"
            f"  Fretting       : {'ON' if sensor.fretting_enabled else 'OFF'}\n"
            "Software\n"
            f"  Scale / Offset : {software.scale:.4f} / {software.offset_m:.3f} m\n"
            f"  Filter         : {software.filter_type.value} (size {software.filter_size})\n"
            f"  EMA / Outlier  : {software.ema_alpha:.2f} / {software.outlier_threshold_m:.3f} m"
        )
        self.graph.draw(snapshot)
        self.root.after(100, self._tick)

    def close(self) -> None:
        if self._closing:
            return
        self._closing = True
        self.recorder.stop()
        self.worker.close()
        self.root.destroy()


def parse_arguments(args=None):
    parser = argparse.ArgumentParser(description="C4001 standalone UART tuning GUI")
    parser.add_argument("--port", default="/dev/ttyAMA0", help="C4001 UART device")
    parser.add_argument("--baud-rate", type=int, default=9600)
    parser.add_argument(
        "--log-directory",
        type=Path,
        default=Path.home() / "mmwave_tuning_logs",
    )
    options, _ros_arguments = parser.parse_known_args(args)
    return options


def main(args=None) -> None:
    if tk is None:
        raise RuntimeError("python3-tk is required to run mmwave_tuning_gui")
    options = parse_arguments(args)
    if options.baud_rate <= 0:
        raise ValueError("baud-rate must be positive")
    sensor_settings = SensorSettings()
    store = TuningStore(sensor_settings)
    recorder = CsvRecorder(options.log_directory.expanduser())
    root = tk.Tk()
    gui_holder: Dict[str, TuningGui] = {}

    def post_status(text: str) -> None:
        gui = gui_holder.get("gui")
        if gui is not None:
            gui.post_status(text)

    worker = C4001UartWorker(
        port=options.port,
        baud_rate=options.baud_rate,
        store=store,
        recorder=recorder,
        status_callback=post_status,
    )
    gui = TuningGui(root, store, worker, recorder, recorder.log_directory)
    gui_holder["gui"] = gui
    print(f"mmwave_tuning_gui: CSV logs: {recorder.log_directory}", flush=True)
    worker.start()
    try:
        root.mainloop()
    except KeyboardInterrupt:
        gui.close()


if __name__ == "__main__":
    main()
