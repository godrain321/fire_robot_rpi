#!/usr/bin/env python3
"""Summary statistics for recorded MQ-135 ADC samples (Stage 7 calibration aid).

This is an OFFLINE, ROS-independent helper. It is not a ROS node, is not part of
any package's ``console_scripts``, and is never started by a robot launch file.
It only turns a file of recorded ``/mq135/raw_adc`` / ``/mq135/filtered_adc``
values into count / mean / median / min / max / std / percentiles so a human can
*look* at each measured environment and choose ``gas_safe_adc`` / ``gas_blocked_adc``.

It deliberately does NOT decide thresholds. The MQ-135 responds to many gases and
varies with the installed sensor and environment; the ADC scalar is NOT ppm. Any
"mean + k*std" number printed below is descriptive only -- the operator picks the
final values from real experiments in a known-safe test setup.

Input formats (blank lines and ``#`` comments ignored):
  * one numeric value per line              (``ros2 topic echo TOPIC --field data --csv``)
  * ``<timestamp>,<value>`` CSV rows        (value taken from --field / last column)
Malformed lines are counted and skipped, never fatal.

Usage:
  python3 tools/analyze_mq135_calibration.py baseline.csv
  python3 tools/analyze_mq135_calibration.py A_indoor.csv B_slightly_up.csv C_high.csv
  python3 tools/analyze_mq135_calibration.py --field 1 --json samples.csv
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Iterable, Sequence

import numpy as np

PERCENTILES = (1, 5, 25, 50, 75, 90, 95, 99)


def parse_samples(lines: Iterable[str], *, field: int | None = None) -> tuple[list[float], int]:
    """Return (values, skipped_line_count). Never raises on bad content."""
    values: list[float] = []
    skipped = 0
    for raw in lines:
        line = raw.strip()
        if not line or line.startswith("#"):
            continue
        parts = [p for p in line.replace(",", " ").split() if p]
        if not parts:
            continue
        token = parts[field] if field is not None and field < len(parts) else parts[-1]
        try:
            value = float(token)
        except ValueError:
            skipped += 1
            continue
        if not np.isfinite(value):
            skipped += 1
            continue
        values.append(value)
    return values, skipped


def summarize(values: Sequence[float]) -> dict:
    array = np.asarray(values, dtype=float)
    if array.size == 0:
        raise ValueError("no numeric samples found")
    stats = {
        "count": int(array.size),
        "mean": float(np.mean(array)),
        "median": float(np.median(array)),
        "min": float(np.min(array)),
        "max": float(np.max(array)),
        # sample standard deviation (ddof=1); 0.0 for a single sample
        "std": float(np.std(array, ddof=1)) if array.size > 1 else 0.0,
    }
    stats["percentiles"] = {
        f"p{p:02d}": float(np.percentile(array, p)) for p in PERCENTILES
    }
    return stats


def _format_block(label: str, stats: dict) -> str:
    lines = [
        f"== {label} ==",
        f"  count : {stats['count']}",
        f"  mean  : {stats['mean']:.2f}",
        f"  median: {stats['median']:.2f}",
        f"  min   : {stats['min']:.2f}",
        f"  max   : {stats['max']:.2f}",
        f"  std   : {stats['std']:.2f}   (sample, ddof=1; descriptive only)",
        "  percentiles: " + "  ".join(
            f"{k}={v:.1f}" for k, v in stats["percentiles"].items()
        ),
    ]
    return "\n".join(lines)


def analyze_paths(paths: Sequence[Path], *, field: int | None = None) -> dict:
    report: dict = {"files": {}, "skipped_lines": {}}
    combined: list[float] = []
    for path in paths:
        text = path.read_text(encoding="utf-8", errors="replace")
        values, skipped = parse_samples(text.splitlines(), field=field)
        report["skipped_lines"][path.name] = skipped
        if values:
            report["files"][path.name] = summarize(values)
            combined.extend(values)
        else:
            report["files"][path.name] = {"count": 0}
    if len(paths) > 1 and combined:
        report["combined"] = summarize(combined)
    return report


_DISCLAIMER = (
    "NOTE: values above are raw ESP32 ADC, NOT ppm. This tool does not choose "
    "gas_safe_adc / gas_blocked_adc. Pick them yourself from these numbers plus "
    "real measurements in a known-safe setup, then pass them as launch args."
)


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("paths", nargs="+", type=Path, help="recorded ADC sample files")
    parser.add_argument(
        "--field", type=int, default=None,
        help="0-based column index of the ADC value (default: last column)",
    )
    parser.add_argument("--json", action="store_true", help="emit machine-readable JSON")
    args = parser.parse_args(argv)

    missing = [str(p) for p in args.paths if not p.is_file()]
    if missing:
        print(f"error: file(s) not found: {', '.join(missing)}", file=sys.stderr)
        return 2

    report = analyze_paths(args.paths, field=args.field)

    empty = [name for name, s in report["files"].items() if s.get("count", 0) == 0]
    if len(empty) == len(report["files"]):
        print("error: no numeric samples in any input file", file=sys.stderr)
        return 1

    if args.json:
        print(json.dumps(report, indent=2, sort_keys=True))
        return 0

    for name, stats in report["files"].items():
        skipped = report["skipped_lines"].get(name, 0)
        if stats.get("count", 0) == 0:
            print(f"== {name} ==\n  (no numeric samples; {skipped} lines skipped)")
            continue
        print(_format_block(name, stats))
        if skipped:
            print(f"  (skipped {skipped} malformed line(s))")
        print()
    if "combined" in report:
        print(_format_block("combined", report["combined"]))
        print()
    print(_DISCLAIMER)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
