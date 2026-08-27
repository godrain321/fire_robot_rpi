from geometry_msgs.msg import Quaternion
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan

from inno_robot_bringup.auto_localization_supervisor import scan_map_overlap_ratio


def map_with_wall() -> OccupancyGrid:
    message = OccupancyGrid()
    message.info.width = 10
    message.info.height = 10
    message.info.resolution = 1.0
    message.info.origin.orientation = Quaternion(w=1.0)
    message.data = [0] * 100
    message.data[5 * 10 + 5] = 100
    return message


def one_beam_scan() -> LaserScan:
    scan = LaserScan()
    scan.angle_min = 0.0
    scan.angle_increment = 1.0
    scan.range_min = 0.05
    scan.range_max = 10.0
    scan.ranges = [5.0]
    return scan


def test_scan_overlap_accepts_endpoint_on_saved_wall():
    ratio, beams = scan_map_overlap_ratio(
        map_with_wall(), one_beam_scan(),
        map_base_pose=(0.5, 5.5, 0.0),
        base_laser_pose=(0.0, 0.0, 0.0),
        tolerance_m=0.0,
        beam_stride=1,
    )
    assert beams == 1
    assert ratio == 1.0


def test_scan_overlap_rejects_endpoint_in_free_space():
    ratio, beams = scan_map_overlap_ratio(
        map_with_wall(), one_beam_scan(),
        map_base_pose=(0.5, 2.5, 0.0),
        base_laser_pose=(0.0, 0.0, 0.0),
        tolerance_m=0.0,
        beam_stride=1,
    )
    assert beams == 1
    assert ratio == 0.0


def test_scan_overlap_rejects_ray_crossing_a_nearer_wall():
    map_message = map_with_wall()
    # The endpoint still lands on the original wall at cell (5, 5), but this
    # candidate pose would require the ray to pass through cell (3, 5).
    map_message.data[5 * 10 + 3] = 100
    ratio, beams = scan_map_overlap_ratio(
        map_message, one_beam_scan(),
        map_base_pose=(0.5, 5.5, 0.0),
        base_laser_pose=(0.0, 0.0, 0.0),
        tolerance_m=0.0,
        beam_stride=1,
    )
    assert beams == 1
    assert ratio == 0.0


def test_scan_overlap_counts_out_of_map_returns_as_mismatches():
    ratio, beams = scan_map_overlap_ratio(
        map_with_wall(), one_beam_scan(),
        map_base_pose=(9.5, 5.5, 0.0),
        base_laser_pose=(0.0, 0.0, 0.0),
        tolerance_m=0.0,
        beam_stride=1,
    )
    assert beams == 1
    assert ratio == 0.0
