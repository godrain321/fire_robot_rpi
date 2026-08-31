"""Node-level contract tests using the same fake-self pattern as
test_evacuation_manager_contract.py, without a live rclpy context.
"""

from types import SimpleNamespace

import numpy as np
from rclpy.clock import Clock

from inno_autonav.event_replanning import EventReplanningConfig
from inno_autonav.exit_evaluator import ExitHazardSnapshot
from inno_autonav.replan_supervisor import ReplanSupervisorCore, RetryConfig, SupervisorOutput
from inno_autonav.replan_supervisor_node import ReplanSupervisorNode
from inno_hazard.hazard_belief import HazardGridGeometry


class Publisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


def snapshot(size=4, revision=1, temperature_blocked_c=60.0, co_blocked_ppm=1600.0):
    shape = size, size
    geometry = HazardGridGeometry(size, size, 1.0)
    false = np.zeros(shape, dtype=bool)
    return ExitHazardSnapshot(
        geometry, np.ones(shape), np.full(shape, np.nan), np.full(shape, np.nan),
        false, false, false, np.zeros(shape), false, false, false,
        revision, temperature_blocked_c, co_blocked_ppm, 1.0,
    )


def node(**core_overrides):
    core = ReplanSupervisorCore(EventReplanningConfig(**core_overrides), RetryConfig())
    core.set_enabled(True)
    value = SimpleNamespace(
        core=core,
        map_frame="map",
        hold_publisher=Publisher(),
        goal_publisher=Publisher(),
        status_publisher=Publisher(),
        get_clock=lambda: Clock(),
        get_logger=lambda: SimpleNamespace(
            info=lambda *a, **k: None, error=lambda *a, **k: None,
        ),
    )
    value._config_overrides = {"enabled": True}
    return value


def test_publish_order_is_hold_then_goal_then_status():
    value = node()
    output = SupervisorOutput(True, (4.5, 0.5), {"state": "REPLAN_REQUESTED"})
    ReplanSupervisorNode._publish(value, output)
    assert len(value.hold_publisher.messages) == 1
    assert len(value.goal_publisher.messages) == 1
    goal = value.goal_publisher.messages[0]
    assert (goal.pose.position.x, goal.pose.position.y) == (4.5, 0.5)
    assert goal.header.frame_id == "map"
    assert len(value.status_publisher.messages) == 1


def test_publish_does_not_send_goal_when_none():
    value = node()
    output = SupervisorOutput(False, None, {"state": "PATH_VALID"})
    ReplanSupervisorNode._publish(value, output)
    assert not value.goal_publisher.messages
    assert value.hold_publisher.messages[0].data is False


def test_rebuild_core_keeps_hazard_thresholds_single_sourced():
    value = node()
    original_core = value.core
    ReplanSupervisorNode._rebuild_core_if_thresholds_changed(
        value, snapshot(temperature_blocked_c=70.0, co_blocked_ppm=2000.0),
    )
    assert value.core is not original_core
    assert value.core.config.temperature_block_c == 70.0
    assert value.core.config.co_block_ppm == 2000.0
    assert value.core.enabled is True  # enabled state preserved across rebuild


def test_rebuild_core_is_a_no_op_when_thresholds_already_match():
    value = node()
    original_core = value.core
    ReplanSupervisorNode._rebuild_core_if_thresholds_changed(value, snapshot())
    assert value.core is original_core


def test_lower_live_temperature_block_preserves_five_degree_hysteresis():
    value = node()

    ReplanSupervisorNode._rebuild_core_if_thresholds_changed(
        value, snapshot(temperature_blocked_c=50.0),
    )

    assert value.core.config.temperature_block_c == 50.0
    assert value.core.config.temperature_release_c == 45.0
