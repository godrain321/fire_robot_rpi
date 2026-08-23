import pytest

from inno_mmwave.mobility_classifier import (
    ASSIST_CHECK,
    MOVING,
    NO_TARGET,
    ROBOT_MOVING,
    SENSOR_OFFLINE,
    STILL_MONITOR,
    MobilityClassifier,
    MobilityConfig,
    human_state_from_mobility,
)


def classifier():
    return MobilityClassifier(MobilityConfig(
        moving_speed_threshold_mps=0.20,
        moving_confirm_samples=3,
        moving_confirm_sec=0.20,
        moving_hold_sec=1.0,
        assist_check_sec=10.0,
        robot_linear_threshold_mps=0.01,
        robot_angular_threshold_rps=0.03,
        robot_settle_sec=2.0,
    ))


def test_public_human_state_contract():
    assert human_state_from_mobility(MOVING) == 'MOVING'
    assert human_state_from_mobility(STILL_MONITOR) == 'STILL'
    assert human_state_from_mobility(ASSIST_CHECK) == 'STILL'
    assert human_state_from_mobility(NO_TARGET) == 'NO_HUMAN'
    assert human_state_from_mobility(ROBOT_MOVING) == 'ROBOT_MOVING'
    assert human_state_from_mobility(SENSOR_OFFLINE) == 'SENSOR_OFFLINE'


def test_state_progresses_without_claiming_medical_condition():
    item = classifier()
    assert item.state(0.0) == SENSOR_OFFLINE
    item.update_sensor_state('ONLINE')
    assert item.state(0.0) == NO_TARGET
    item.update_target(True, 0.0, 1.0)
    # A newly detected stationary target starts as unknown/still, never as
    # fabricated motion.
    assert item.state(1.0) == STILL_MONITOR
    assert item.state(3.0) == STILL_MONITOR
    assert item.state(11.1) == ASSIST_CHECK


def test_new_motion_resets_stillness_timer():
    item = classifier()
    item.update_sensor_state('ONLINE')
    item.update_target(True, 0.0, 0.0)
    assert item.state(5.0) == STILL_MONITOR
    item.update_speed(-0.25, 6.0)
    item.update_speed(-0.26, 6.1)
    assert item.state(6.1) == STILL_MONITOR
    item.update_speed(-0.24, 6.2)
    assert item.state(6.5) == MOVING
    assert item.still_duration(6.5) == 0.0


def test_isolated_speed_spikes_never_claim_motion_or_reset_assist_timer():
    item = classifier()
    item.update_sensor_state('ONLINE')
    item.update_presence(True, 0.0)

    # Large but isolated samples model the C4001 stationary-target spikes.
    item.update_speed(2.8, 2.0)
    item.update_speed(0.01, 2.1)
    item.update_speed(-4.0, 5.0)
    item.update_speed(0.0, 5.1)
    item.update_speed(8.0, 9.0)
    assert item.state(9.0) == STILL_MONITOR
    assert item.state(10.1) == ASSIST_CHECK


def test_motion_requires_samples_and_minimum_elapsed_time():
    item = classifier()
    item.update_sensor_state('ONLINE')
    item.update_presence(True, 0.0)

    # Three callbacks delivered in a burst do not satisfy the time criterion.
    item.update_speed(0.4, 1.00)
    item.update_speed(0.4, 1.01)
    item.update_speed(0.4, 1.02)
    assert item.state(1.02) == STILL_MONITOR
    item.update_speed(0.4, 1.20)
    assert item.state(1.20) == MOVING


def test_non_finite_speed_is_not_motion_evidence():
    item = classifier()
    item.update_sensor_state('ONLINE')
    item.update_presence(True, 0.0)
    item.update_speed(0.5, 1.0)
    item.update_speed(float('nan'), 1.1)
    item.update_speed(0.5, 1.2)
    item.update_speed(0.5, 1.3)
    assert item.state(1.3) == STILL_MONITOR


def test_robot_motion_suspends_person_mobility_inference_until_settled():
    item = classifier()
    item.update_sensor_state('ONLINE')
    item.update_target(True, 0.0, 0.0)
    item.update_robot_command(0.05, 0.0, 5.0)
    assert item.state(6.9) == ROBOT_MOVING
    assert item.state(7.1) == STILL_MONITOR
    assert item.still_duration(7.1) == pytest.approx(0.1)

    # Self-motion speed samples are ignored and cannot become MOVING later.
    item.update_speed(3.0, 5.2)
    item.update_speed(3.0, 5.4)
    item.update_speed(3.0, 5.6)
    assert item.state(7.1) == STILL_MONITOR


def test_disconnect_clears_old_presence_and_assist_timer():
    item = classifier()
    item.update_sensor_state('ONLINE')
    item.update_presence(True, 0.0)
    assert item.state(11.0) == ASSIST_CHECK
    item.update_sensor_state('OFFLINE')
    item.update_sensor_state('ONLINE')
    assert item.state(11.1) == NO_TARGET
    item.update_presence(True, 11.2)
    assert item.state(11.2) == STILL_MONITOR


def test_invalid_configuration_is_rejected():
    with pytest.raises(ValueError):
        MobilityClassifier(MobilityConfig(
            moving_hold_sec=2.0,
            assist_check_sec=1.0,
        ))
    with pytest.raises(ValueError):
        MobilityClassifier(MobilityConfig(moving_confirm_samples=1))
