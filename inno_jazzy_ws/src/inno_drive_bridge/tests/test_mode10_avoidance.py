import math
import sys
import unittest
from pathlib import Path
from types import SimpleNamespace

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from inno_drive_bridge.mode10_avoidance_core import (  # noqa: E402
    Mode10Avoidance, sector_clearance,
)
from inno_drive_bridge.ultrasonic_protocol import parse_ultrasonic_packet  # noqa: E402


class Mode10AvoidanceTest(unittest.TestCase):
    def test_sonar_packet_and_invalid_reading(self):
        self.assertAlmostEqual(parse_ultrasonic_packet('US,100,49.90,1'), 0.499)
        self.assertTrue(math.isnan(parse_ultrasonic_packet('US,200,0.00,0')))
        for packet in ('US,1,abc,1', 'US,1,500,1', 'US,1,49,2', 'US,1,49'):
            with self.assertRaises(ValueError):
                parse_ultrasonic_packet(packet)

    def test_below_threshold_stops_then_turns_after_one_second(self):
        control = Mode10Avoidance()
        control.on_scan(2.0, 1.5, 0.8, 0.0)
        control.on_distance(1.0, 0.0)
        self.assertEqual(control.command(0.0), (0.06, 0.0, 'CRUISE'))
        for now in (0.1, 0.3, 0.5, 0.7, 0.9):
            control.on_distance(0.49, now)
            control.on_scan(0.49, 1.5, 0.8, now)
            self.assertEqual(control.command(now)[0:2], (0.0, 0.0))
        control.on_distance(0.49, 1.1)
        control.on_scan(0.49, 1.5, 0.8, 1.1)
        self.assertEqual(control.command(1.1), (0.0, 0.35, 'TURN'))

    def test_clear_sample_resets_continuous_one_second_trigger(self):
        control = Mode10Avoidance()
        for now, distance in ((0.0, 0.49), (0.5, 0.60), (0.6, 0.49), (1.1, 0.49)):
            control.on_distance(distance, now)
            control.on_scan(distance, 2.0, 2.0, now)
            control.command(now)
        self.assertEqual(control.state, 'CONFIRM')

    def test_invalid_or_stale_sensor_never_drives(self):
        control = Mode10Avoidance()
        control.on_scan(2.0, 2.0, 2.0, 0.0)
        control.on_distance(math.nan, 0.0)
        self.assertEqual(control.command(0.0), (0.0, 0.0, 'WAIT_SENSOR'))
        control.on_distance(1.0, 0.1)
        self.assertEqual(control.command(0.6), (0.0, 0.0, 'WAIT_SENSOR'))

    def test_blocked_side_and_emergency_distance_latch_stop(self):
        control = Mode10Avoidance()
        for now in (0.0, 0.25, 0.50, 0.75, 1.0, 1.1):
            control.on_distance(0.40, now)
            control.on_scan(0.4, 0.5, 0.6, now)
        self.assertEqual(control.command(1.1), (0.0, 0.0, 'BLOCKED'))
        control.on_distance(2.0, 1.2)
        control.on_scan(2.0, 2.0, 2.0, 1.2)
        self.assertEqual(control.command(1.2), (0.0, 0.0, 'BLOCKED'))
        control.set_armed(True)
        control.on_distance(0.15, 1.3)
        self.assertEqual(control.command(1.3), (0.0, 0.0, 'BLOCKED'))

    def test_turn_resumes_only_after_both_sensors_clear(self):
        control = Mode10Avoidance()
        for now in (0.0, 0.25, 0.50, 0.75, 1.0):
            control.on_distance(0.4, now)
            control.on_scan(0.4, 1.5, 0.8, now)
        self.assertEqual(control.command(1.0)[2], 'TURN')
        control.on_distance(0.75, 1.4)
        control.on_scan(0.6, 1.5, 0.8, 1.4)
        self.assertEqual(control.command(1.4)[2], 'TURN')
        for now in (2.0, 2.4):
            control.on_distance(0.75, now)
            control.on_scan(0.8, 1.5, 0.8, now)
            result = control.command(now)
        self.assertEqual(result, (0.06, 0.0, 'CRUISE'))

    def test_lidar_sector_uses_nearest_point_and_missing_sector_blocks(self):
        scan = SimpleNamespace(
            angle_min=math.radians(-90),
            angle_increment=math.radians(45),
            range_max=4.0,
            ranges=[1.0, 2.0, 0.4, math.inf, 3.0],
        )
        self.assertEqual(sector_clearance(scan, -25, 25), 0.4)
        self.assertEqual(sector_clearance(scan, 35, 100), 3.0)
        self.assertIsNone(sector_clearance(scan, 120, 150))


if __name__ == '__main__':
    unittest.main()
