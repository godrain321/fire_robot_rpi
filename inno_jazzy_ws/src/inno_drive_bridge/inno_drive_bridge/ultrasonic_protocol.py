"""ESP32 HC-SR04 serial packet validation, independent of ROS."""

import math


def parse_ultrasonic_packet(line):
    """Return distance in metres, or NaN for a failed HC-SR04 reading."""
    fields = line.split(',')
    if len(fields) != 4 or fields[0] != 'US':
        raise ValueError('expected US,<millis>,<distance_cm>,<valid>')
    int(fields[1])
    distance_cm = float(fields[2])
    if fields[3] not in ('0', '1'):
        raise ValueError('ultrasonic valid flag must be 0 or 1')
    if fields[3] == '0':
        return math.nan
    if not math.isfinite(distance_cm) or not 2.0 <= distance_cm <= 400.0:
        raise ValueError('ultrasonic distance outside sensor range')
    return distance_cm / 100.0
