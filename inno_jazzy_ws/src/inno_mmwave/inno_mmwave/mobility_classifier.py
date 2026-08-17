"""Pure, testable mobility-state inference for C4001 target samples.

The C4001 does not report posture, consciousness, or fall events.  These
states intentionally describe observed motion only, and ASSIST_CHECK remains
an operator prompt rather than a medical conclusion.
"""

from dataclasses import dataclass
import math


SENSOR_OFFLINE = 'SENSOR_OFFLINE'
ROBOT_MOVING = 'ROBOT_MOVING'
NO_TARGET = 'NO_TARGET'
MOVING = 'MOVING'
STILL_MONITOR = 'STILL_MONITOR'
ASSIST_CHECK = 'ASSIST_CHECK'


@dataclass(frozen=True)
class MobilityConfig:
    # A motion decision needs both a meaningful filtered radial speed and
    # several samples spanning time. C4001 occasionally emits a large single
    # speed sample for a stationary target; one sample must never reset the
    # stillness/assistance timer.
    moving_speed_threshold_mps: float = 0.20
    moving_confirm_samples: int = 3
    moving_confirm_sec: float = 0.20
    moving_hold_sec: float = 1.0
    assist_check_sec: float = 10.0
    robot_linear_threshold_mps: float = 0.01
    robot_angular_threshold_rps: float = 0.03
    robot_settle_sec: float = 2.0

    def validate(self) -> None:
        values = (
            self.moving_speed_threshold_mps,
            self.moving_confirm_sec,
            self.moving_hold_sec,
            self.assist_check_sec,
            self.robot_linear_threshold_mps,
            self.robot_angular_threshold_rps,
            self.robot_settle_sec,
        )
        if any(value < 0.0 for value in values):
            raise ValueError('mobility thresholds must not be negative')
        if self.moving_speed_threshold_mps <= 0.0:
            raise ValueError('moving_speed_threshold_mps must be greater than zero')
        if self.moving_confirm_samples < 2:
            raise ValueError('moving_confirm_samples must be at least two')
        if self.assist_check_sec <= self.moving_hold_sec:
            raise ValueError('assist_check_sec must be greater than moving_hold_sec')


class MobilityClassifier:
    """Infer display state while explicitly accounting for robot self-motion."""

    def __init__(self, config: MobilityConfig) -> None:
        config.validate()
        self.config = config
        self.sensor_online = False
        self.presence = False
        self.speed_mps = 0.0
        self.presence_since = float('-inf')
        self.last_person_motion = float('-inf')
        self.last_robot_motion = float('-inf')
        self._motion_evidence_count = 0
        self._motion_evidence_since = float('-inf')

    def update_sensor_state(self, state: str) -> None:
        online = state.strip().upper() == 'ONLINE'
        if not online:
            # Never carry an old stillness timer across a disconnect. A fresh
            # filtered presence sample starts a new observation period.
            self._clear_target()
        self.sensor_online = online

    def _reset_motion_evidence(self) -> None:
        self._motion_evidence_count = 0
        self._motion_evidence_since = float('-inf')

    def _clear_target(self) -> None:
        self.presence = False
        self.speed_mps = 0.0
        self.presence_since = float('-inf')
        self.last_person_motion = float('-inf')
        self._reset_motion_evidence()

    def update_robot_command(
        self, linear_mps: float, angular_rps: float, now: float
    ) -> None:
        if (
            abs(float(linear_mps)) >= self.config.robot_linear_threshold_mps
            or abs(float(angular_rps)) >= self.config.robot_angular_threshold_rps
        ):
            self.last_robot_motion = float(now)
            # Radar radial speed while the platform moves is not evidence of
            # person motion. Require fresh evidence after the robot settles.
            self._reset_motion_evidence()

    def update_presence(self, presence: bool, now: float) -> None:
        now = float(now)
        presence = bool(presence)
        if not presence:
            self._clear_target()
            return

        if not self.presence:
            self.presence = True
            self.presence_since = now
            self.last_person_motion = float('-inf')
            self._reset_motion_evidence()

    def update_speed(self, speed_mps: float, now: float) -> None:
        now = float(now)
        speed_mps = float(speed_mps)
        if not math.isfinite(speed_mps):
            self._reset_motion_evidence()
            return
        self.speed_mps = speed_mps

        if not self.presence:
            self._reset_motion_evidence()
            return
        if now - self.last_robot_motion < self.config.robot_settle_sec:
            self._reset_motion_evidence()
            return

        if abs(speed_mps) < self.config.moving_speed_threshold_mps:
            self._reset_motion_evidence()
            return

        if self._motion_evidence_count == 0:
            self._motion_evidence_since = now
        self._motion_evidence_count += 1
        evidence_age = now - self._motion_evidence_since
        if (
            self._motion_evidence_count >= self.config.moving_confirm_samples
            and evidence_age + 1e-9 >= self.config.moving_confirm_sec
        ):
            # Only confirmed, sustained motion may reset the assistance timer.
            self.last_person_motion = now

    def update_target(self, presence: bool, speed_mps: float, now: float) -> None:
        """Convenience API for callers that receive an atomic target sample."""
        self.update_presence(presence, now)
        if presence:
            self.update_speed(speed_mps, now)

    def _still_reference(self) -> float:
        reference = self.presence_since
        if math.isfinite(self.last_person_motion):
            reference = max(reference, self.last_person_motion)
        if math.isfinite(self.last_robot_motion):
            # Observations made while the robot moves or settles are invalid.
            reference = max(
                reference,
                self.last_robot_motion + self.config.robot_settle_sec,
            )
        return reference

    def state(self, now: float) -> str:
        now = float(now)
        if not self.sensor_online:
            return SENSOR_OFFLINE
        if now - self.last_robot_motion < self.config.robot_settle_sec:
            return ROBOT_MOVING
        if not self.presence:
            return NO_TARGET
        if (
            math.isfinite(self.last_person_motion)
            and now - self.last_person_motion <= self.config.moving_hold_sec
        ):
            return MOVING
        still_for = max(0.0, now - self._still_reference())
        if still_for >= self.config.assist_check_sec:
            return ASSIST_CHECK
        return STILL_MONITOR

    def still_duration(self, now: float) -> float:
        if self.state(now) not in (STILL_MONITOR, ASSIST_CHECK):
            return 0.0
        return max(0.0, float(now) - self._still_reference())
