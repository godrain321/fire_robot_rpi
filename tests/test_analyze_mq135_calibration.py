"""Pure-function tests for the offline MQ-135 calibration stats helper.

Covers Stage 7 section 18: mean/median/min/max/std, empty input, malformed
samples, and that the tool never decides thresholds on its own.
"""

import json
import math
import subprocess
import sys
from pathlib import Path

import pytest

from tools.analyze_mq135_calibration import (
    _DISCLAIMER,
    analyze_paths,
    main,
    parse_samples,
    summarize,
)

SCRIPT = Path(__file__).resolve().parents[1] / "tools" / "analyze_mq135_calibration.py"


# -- parsing --------------------------------------------------------------
def test_parses_one_value_per_line():
    values, skipped = parse_samples(["1800", "1810.5", "1795\n"])
    assert values == [1800.0, 1810.5, 1795.0]
    assert skipped == 0


def test_parses_timestamp_value_csv_taking_last_column():
    values, skipped = parse_samples(["1699999999.1,1851", "1699999999.3, 1849"])
    assert values == [1851.0, 1849.0]
    assert skipped == 0


def test_field_index_selects_column():
    values, _ = parse_samples(["10,1851,x", "11,1849,y"], field=1)
    assert values == [1851.0, 1849.0]


def test_ignores_blank_and_comment_lines():
    values, skipped = parse_samples(["", "  ", "# header", "1800", "\t", "1801"])
    assert values == [1800.0, 1801.0]
    assert skipped == 0


def test_counts_malformed_lines_without_crashing():
    values, skipped = parse_samples(["1800", "abc", "12.3.4", "nan", "inf", "1802"])
    assert values == [1800.0, 1802.0]
    assert skipped == 4  # abc, 12.3.4, nan, inf


# -- statistics ----------------------------------------------------------
def test_summarize_basic_stats():
    stats = summarize([10, 20, 30, 40, 50])
    assert stats["count"] == 5
    assert stats["mean"] == 30.0
    assert stats["median"] == 30.0
    assert stats["min"] == 10.0
    assert stats["max"] == 50.0
    assert stats["std"] == pytest.approx(math.sqrt(250.0))  # ddof=1
    assert stats["percentiles"]["p50"] == 30.0


def test_summarize_single_sample_has_zero_std():
    stats = summarize([1817.3])
    assert stats["count"] == 1 and stats["std"] == 0.0
    assert stats["min"] == stats["max"] == 1817.3


def test_summarize_empty_raises():
    with pytest.raises(ValueError):
        summarize([])


# -- file-level analysis + combined -------------------------------------
def test_analyze_paths_per_file_and_combined(tmp_path):
    a = tmp_path / "a.csv"
    a.write_text("1000\n1002\n998\n")
    b = tmp_path / "b.csv"
    b.write_text("2000\n2010\n1990\n")
    report = analyze_paths([a, b])
    assert report["files"]["a.csv"]["mean"] == pytest.approx(1000.0)
    assert report["files"]["b.csv"]["mean"] == pytest.approx(2000.0)
    assert report["combined"]["count"] == 6
    assert report["combined"]["min"] == 998.0 and report["combined"]["max"] == 2010.0


def test_analyze_paths_reports_empty_file(tmp_path):
    empty = tmp_path / "e.csv"
    empty.write_text("# only comments\n\n")
    report = analyze_paths([empty])
    assert report["files"]["e.csv"] == {"count": 0}


# -- CLI ---------------------------------------------------------------
def test_cli_json_output(tmp_path, capsys):
    f = tmp_path / "s.csv"
    f.write_text("1500\n1520\n1490\n1510\n")
    rc = main([str(f), "--json"])
    assert rc == 0
    payload = json.loads(capsys.readouterr().out)
    assert payload["files"]["s.csv"]["count"] == 4


def test_cli_missing_file_returns_2(tmp_path):
    assert main([str(tmp_path / "nope.csv")]) == 2


def test_cli_all_empty_returns_1(tmp_path, capsys):
    f = tmp_path / "e.csv"
    f.write_text("\n#\n")
    assert main([str(f)]) == 1


def test_cli_text_output_carries_disclaimer_and_no_auto_threshold(tmp_path, capsys):
    f = tmp_path / "s.csv"
    f.write_text("\n".join(str(v) for v in range(1000, 1100)) + "\n")
    main([str(f)])
    out = capsys.readouterr().out
    assert _DISCLAIMER in out
    # the tool must not emit a chosen threshold parameter
    assert "gas_safe_adc =" not in out and "gas_blocked_adc =" not in out


def test_script_runs_as_module(tmp_path):
    f = tmp_path / "s.csv"
    f.write_text("1800\n1801\n1802\n")
    result = subprocess.run(
        [sys.executable, str(SCRIPT), str(f)],
        capture_output=True, text=True, check=True,
    )
    assert "mean" in result.stdout and "NOT ppm" in result.stdout
