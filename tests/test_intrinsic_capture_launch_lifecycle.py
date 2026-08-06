"""Static lifecycle checks for the ROS intrinsic-capture launch file."""

import ast
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
LAUNCH_FILE = (
    ROOT
    / "camera_ws/src/fire_robot_camera_calibration/launch"
    / "intrinsic_capture.launch.py"
)


def call_name(call: ast.Call) -> str | None:
    """Return the simple function name for a call, when available."""
    return call.func.id if isinstance(call.func, ast.Name) else None


def test_capture_exit_emits_launch_shutdown_before_processes_start():
    source = LAUNCH_FILE.read_text(encoding="utf-8")
    tree = ast.parse(source)

    exit_handlers = [
        node
        for node in ast.walk(tree)
        if isinstance(node, ast.Call) and call_name(node) == "OnProcessExit"
    ]
    assert len(exit_handlers) == 1
    handler = exit_handlers[0]
    target = next(
        keyword.value
        for keyword in handler.keywords
        if keyword.arg == "target_action"
    )
    assert isinstance(target, ast.Name)
    assert target.id == "capture"
    assert any(
        isinstance(node, ast.Call) and call_name(node) == "Shutdown"
        for node in ast.walk(handler)
    )

    compact_source = " ".join(source.split())
    assert (
        "[shutdown_when_capture_exits, camera_ros_node(), capture]"
        in compact_source
    )


def test_launch_and_capture_docstrings_describe_rational_dataset():
    launch_tree = ast.parse(LAUNCH_FILE.read_text(encoding="utf-8"))
    launch_docstring = ast.get_docstring(launch_tree) or ""
    assert "Rational" in launch_docstring
    assert "fisheye" not in launch_docstring.lower()

    node_file = (
        ROOT
        / "camera_ws/src/fire_robot_camera_calibration"
        / "fire_robot_camera_calibration/guided_capture_node.py"
    )
    node_tree = ast.parse(node_file.read_text(encoding="utf-8"))
    node_docstring = ast.get_docstring(node_tree) or ""
    assert "Rational Polynomial" in node_docstring
    assert "fisheye" not in node_docstring.lower()
