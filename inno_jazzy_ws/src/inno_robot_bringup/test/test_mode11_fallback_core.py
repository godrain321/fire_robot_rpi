"""Mode 11's scan-free dead reckoning and RF2O handover invariants."""

import math

import pytest

from inno_robot_bringup.mode11_fallback_core import EncoderImuFallback, Pose2D


def estimator(*, yaw=0.0, left_sign=-1, right_sign=1):
    fallback = EncoderImuFallback(left_sign=left_sign, right_sign=right_sign)
    seed = Pose2D(4.2, 1.8, yaw)
    assert fallback.start(seed, (100, -100), 10.0, 10.0) == seed
    return fallback


def test_blackout_starts_at_exact_rf2o_pose_without_counting_old_ticks():
    fallback = estimator(yaw=math.pi / 6)

    assert fallback.pose == Pose2D(4.2, 1.8, math.pi / 6)
    assert fallback.on_ticks((100, -100), 10.2, imu_fresh=True)
    assert fallback.pose == Pose2D(4.2, 1.8, math.pi / 6)
    assert fallback.linear_speed == 0.0


def test_signed_motor_steps_move_forward_instead_of_canceling():
    fallback = estimator()

    # Existing drive configuration reverses left motor commands. Both wheels
    # have moved ten physical steps forward in this signed telemetry pair.
    assert fallback.on_ticks((90, -90), 10.2, imu_fresh=True)

    expected_distance = 10 * 2 * math.pi * 0.04 / (200 * 8)
    assert fallback.pose.x == pytest.approx(4.2 + expected_distance)
    assert fallback.pose.y == pytest.approx(1.8)
    assert fallback.linear_speed == pytest.approx(expected_distance / 0.2)


def test_gyro_z_sets_yaw_and_midpoint_heading_for_wheel_distance():
    fallback = estimator(yaw=math.pi / 6)

    assert fallback.on_imu(1.0, 10.2)
    assert fallback.on_ticks((80, -80), 10.2, imu_fresh=True)

    distance = 20 * fallback.metres_per_step
    assert fallback.pose.yaw == pytest.approx(math.pi / 6 + 0.2)
    assert fallback.pose.x == pytest.approx(4.2 + distance * math.cos(math.pi / 6 + 0.1))
    assert fallback.pose.y == pytest.approx(1.8 + distance * math.sin(math.pi / 6 + 0.1))
    assert fallback.angular_speed == pytest.approx(1.0)


def test_nonpositive_and_stale_imu_intervals_do_not_rotate_pose():
    fallback = estimator()

    assert not fallback.on_imu(1.0, 10.0)
    assert not fallback.on_imu(1.0, 10.5)
    assert fallback.pose.yaw == 0.0
    assert fallback.angular_speed == 0.0
    assert fallback.on_imu(1.0, 10.6)
    assert fallback.pose.yaw == pytest.approx(0.1)


def test_encoder_timeout_holds_yaw_until_both_sensors_recover():
    fallback = estimator()
    assert not fallback.on_imu(1.0, 10.2, encoder_fresh=False)
    assert fallback.pose == Pose2D(4.2, 1.8, 0.0)
    assert fallback.on_imu(1.0, 10.3, encoder_fresh=True)
    assert fallback.pose.yaw == pytest.approx(0.1)


def test_bad_tick_intervals_and_stale_imu_do_not_integrate_distance():
    fallback = estimator()

    assert not fallback.on_ticks((90, -90), 10.0, imu_fresh=True)
    assert not fallback.on_ticks((80, -80), 10.2, imu_fresh=False)
    assert not fallback.on_ticks((70, -70), 11.0, imu_fresh=True)
    assert fallback.pose == Pose2D(4.2, 1.8, 0.0)
    assert fallback.linear_speed == 0.0

    # Dropped intervals must not be integrated later as one giant jump.
    assert fallback.on_ticks((60, -60), 11.2, imu_fresh=True)
    assert fallback.pose.x == pytest.approx(4.2 + 10 * fallback.metres_per_step)


def test_implausible_tick_jump_is_rejected_and_not_carried_forward():
    fallback = estimator()

    assert not fallback.on_ticks((-5000, 5000), 10.2, imu_fresh=True)
    assert fallback.pose == Pose2D(4.2, 1.8, 0.0)
    assert fallback.linear_speed == 0.0

    assert fallback.on_ticks((-5010, 5010), 10.4, imu_fresh=True)
    assert fallback.pose.x == pytest.approx(4.2 + 10 * fallback.metres_per_step)


def test_nonfinite_transition_and_imu_values_are_rejected():
    fallback = EncoderImuFallback()

    with pytest.raises(ValueError):
        fallback.start(Pose2D(float('nan'), 0.0, 0.0), (0, 0), 10.0, 10.0)
    assert not fallback.active

    fallback.start(Pose2D(4.2, 1.8, 0.0), (0, 0), 10.0, 10.0)
    assert not fallback.on_imu(float('inf'), 10.1)
    assert not fallback.on_imu(1.0, float('nan'))
    assert fallback.pose == Pose2D(4.2, 1.8, 0.0)


def test_nonfinite_tick_values_cannot_change_pose():
    fallback = estimator()

    assert not fallback.on_ticks((float('nan'), 0), 10.2, imu_fresh=True)
    assert not fallback.on_ticks((0, float('inf')), 10.4, imu_fresh=True)
    assert fallback.pose == Pose2D(4.2, 1.8, 0.0)


def test_yaw_wrap_does_not_reverse_midpoint_translation():
    fallback = estimator(yaw=math.pi - 0.05)

    assert fallback.on_imu(1.0, 10.1)
    assert fallback.pose.yaw == pytest.approx(-math.pi + 0.05)
    assert fallback.on_ticks((90, -90), 10.2, imu_fresh=True)

    distance = 10 * fallback.metres_per_step
    assert fallback.pose.x == pytest.approx(4.2 - distance)
    assert fallback.pose.y == pytest.approx(1.8, abs=1e-12)
