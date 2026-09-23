"""Conservative front-sonar trigger and LiDAR-checked reactive steering."""

import math


def sector_clearance(scan, start_deg, end_deg):
    """Minimum range in a scan sector; no usable rays means unavailable."""
    if not math.isfinite(scan.angle_increment) or scan.angle_increment == 0:
        return None
    values = []
    for index, raw in enumerate(scan.ranges):
        angle = math.degrees(scan.angle_min + index * scan.angle_increment)
        angle = (angle + 180.0) % 360.0 - 180.0
        if not start_deg <= angle <= end_deg:
            continue
        if math.isnan(raw):
            continue
        if math.isinf(raw) and raw > 0:
            values.append(float(scan.range_max))
        elif math.isfinite(raw):
            values.append(max(0.0, min(float(raw), float(scan.range_max))))
    return min(values) if values else None


class Mode10Avoidance:
    def __init__(self, *, trigger_m=0.50, clear_m=0.70,
                 hold_sec=1.0, sample_timeout_sec=0.35,
                 scan_timeout_sec=0.50, cruise_mps=0.06,
                 turn_radps=0.35, side_clearance_m=0.70,
                 front_clearance_m=0.70, emergency_m=0.20,
                 minimum_turn_sec=1.0, clear_hold_sec=0.3,
                 maximum_turn_sec=8.0):
        if not (0 < emergency_m < trigger_m < clear_m):
            raise ValueError('expected emergency < trigger < clear distances')
        if min(hold_sec, sample_timeout_sec, scan_timeout_sec, cruise_mps,
               turn_radps, side_clearance_m, front_clearance_m,
               minimum_turn_sec, clear_hold_sec, maximum_turn_sec) <= 0:
            raise ValueError('mode 10 parameters must be positive')
        self.trigger_m = trigger_m
        self.clear_m = clear_m
        self.hold_sec = hold_sec
        self.sample_timeout_sec = sample_timeout_sec
        self.scan_timeout_sec = scan_timeout_sec
        self.cruise_mps = cruise_mps
        self.turn_radps = turn_radps
        self.side_clearance_m = side_clearance_m
        self.front_clearance_m = front_clearance_m
        self.emergency_m = emergency_m
        self.minimum_turn_sec = minimum_turn_sec
        self.clear_hold_sec = clear_hold_sec
        self.maximum_turn_sec = maximum_turn_sec
        self.armed = True
        self.state = 'WAIT_SENSOR'
        self.distance_m = None
        self.distance_at = None
        self.below_since = None
        self.scan_at = None
        self.front_m = None
        self.left_m = None
        self.right_m = None
        self.turn_direction = 0
        self.turn_started = None
        self.clear_since = None

    def set_armed(self, armed):
        self.armed = bool(armed)
        self.state = 'WAIT_SENSOR' if self.armed else 'STOPPED'
        self.below_since = None
        self.turn_direction = 0
        self.turn_started = None
        self.clear_since = None

    def on_distance(self, distance_m, now):
        if self.distance_at is not None and now - self.distance_at > self.sample_timeout_sec:
            self.below_since = None
        self.distance_at = now
        if distance_m is None or not math.isfinite(distance_m) or not 0.02 <= distance_m <= 4.0:
            self.distance_m = None
            self.below_since = None
            return
        self.distance_m = distance_m
        if distance_m < self.trigger_m:
            if self.below_since is None:
                self.below_since = now
        else:
            self.below_since = None

    def on_scan(self, front_m, left_m, right_m, now):
        self.front_m = front_m
        self.left_m = left_m
        self.right_m = right_m
        self.scan_at = now

    def command(self, now):
        """Return (linear m/s, angular rad/s, state); every unsafe input means zero."""
        if not self.armed:
            return 0.0, 0.0, 'STOPPED'
        if self.state == 'BLOCKED':
            return 0.0, 0.0, 'BLOCKED'
        if (self.distance_m is None or self.distance_at is None
                or now - self.distance_at > self.sample_timeout_sec
                or self.scan_at is None
                or now - self.scan_at > self.scan_timeout_sec
                or any(value is None for value in
                       (self.front_m, self.left_m, self.right_m))):
            self.state = 'WAIT_SENSOR'
            self.turn_direction = 0
            self.turn_started = None
            self.clear_since = None
            return 0.0, 0.0, self.state
        if self.distance_m <= self.emergency_m:
            self.state = 'BLOCKED'
            return 0.0, 0.0, self.state

        if self.state == 'TURN':
            side = self.left_m if self.turn_direction > 0 else self.right_m
            if side < self.side_clearance_m or now - self.turn_started > self.maximum_turn_sec:
                self.state = 'BLOCKED'
                return 0.0, 0.0, self.state
            clear = (self.distance_m >= self.clear_m
                     and self.front_m >= self.front_clearance_m)
            if clear:
                if self.clear_since is None:
                    self.clear_since = now
                if (now - self.turn_started >= self.minimum_turn_sec
                        and now - self.clear_since >= self.clear_hold_sec):
                    self.state = 'CRUISE'
                    self.turn_direction = 0
                    return self.cruise_mps, 0.0, self.state
            else:
                self.clear_since = None
            return 0.0, self.turn_direction * self.turn_radps, 'TURN'

        if self.distance_m < self.trigger_m:
            if self.below_since is None or now - self.below_since < self.hold_sec:
                self.state = 'CONFIRM'
                return 0.0, 0.0, self.state
            left_ok = self.left_m >= self.side_clearance_m
            right_ok = self.right_m >= self.side_clearance_m
            if not (left_ok or right_ok):
                self.state = 'BLOCKED'
                return 0.0, 0.0, self.state
            self.turn_direction = 1 if left_ok and (
                not right_ok or self.left_m >= self.right_m) else -1
            self.turn_started = now
            self.clear_since = None
            self.state = 'TURN'
            return 0.0, self.turn_direction * self.turn_radps, self.state

        if self.front_m < self.front_clearance_m:
            self.state = 'WAIT_FRONT_CLEAR'
            return 0.0, 0.0, self.state
        self.state = 'CRUISE'
        return self.cruise_mps, 0.0, self.state
