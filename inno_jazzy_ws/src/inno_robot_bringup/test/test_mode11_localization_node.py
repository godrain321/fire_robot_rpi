"""Mode 11 blackout callback tests without starting DDS or a ROS node."""

import math

from builtin_interfaces.msg import Time
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, LaserScan
from std_msgs.msg import Empty, Int64MultiArray

from inno_robot_bringup.mode11_fallback_core import (
    EncoderImuFallback, Mode11State, Pose2D,
)
from inno_robot_bringup.mode11_localization_node import (
    Mode11Localization, fill_yaw, yaw_from_quaternion,
)


class DummyPublisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


class DummyTfBroadcaster:
    def __init__(self):
        self.transforms = []

    def sendTransform(self, transform):
        self.transforms.append(transform)


class DummyLogger:
    def __init__(self):
        self.infos = []
        self.warnings = []
        self.errors = []

    def info(self, message):
        self.infos.append(message)

    def warning(self, message):
        self.warnings.append(message)

    def error(self, message):
        self.errors.append(message)


class DummyNow:
    def __init__(self, seconds):
        self.nanoseconds = round(seconds * 1e9)

    def to_msg(self):
        seconds, nanoseconds = divmod(self.nanoseconds, 1_000_000_000)
        return Time(sec=seconds, nanosec=nanoseconds)


class DummyClock:
    def __init__(self, seconds):
        self.seconds = seconds

    def now(self):
        return DummyNow(self.seconds)


def make_node(*, imu_topic='/tested/imu', allow_virtual_step_counts=True):
    # Only exercise the callbacks; Node.__init__ would create DDS entities.
    node = object.__new__(Mode11Localization)
    node.state = Mode11State.NORMAL
    node.rf2o_pose = None
    node.rf2o_message = None
    node.rf2o_receipt = None
    node.ticks = None
    node.ticks_receipt = None
    node.tick_samples = 0
    node.imu_stamp = None
    node.imu_receipt = None
    node.imu_angular_z = None
    node.imu_interval_ok = False
    node.map_to_odom = Pose2D(1.0, 2.0, 0.0)
    node.last_sensor_warning = None
    node.rf2o_stale_warning = False
    node.imu_topic = imu_topic
    node.encoder_topic = '/wheel_ticks'
    node.allow_virtual_step_counts = allow_virtual_step_counts
    node.imu_yaw_sign = 1
    node.require_map_alignment = True
    node.rf2o_timeout = 0.5
    node.encoder_timeout = 0.6
    node.imu_timeout = 0.35
    node.scan_timeout = 0.5
    node.fallback = EncoderImuFallback(left_sign=-1, right_sign=1)
    node.scan_publisher = DummyPublisher()
    node.odom_publisher = DummyPublisher()
    node.state_publisher = DummyPublisher()
    node.body_publisher = DummyPublisher()
    node.tf_broadcaster = DummyTfBroadcaster()
    node.logger = DummyLogger()
    node.get_logger = lambda: node.logger
    node.clock = DummyClock(100.1)
    node.get_clock = lambda: node.clock
    node._ros_seconds = lambda: node.clock.seconds
    return node


def rf2o_message(*, x=4.2, y=1.8, yaw=math.pi / 6):
    odom = Odometry()
    odom.header.stamp = Time(sec=100)
    odom.header.frame_id = 'odom'
    odom.child_frame_id = 'base_link'
    odom.pose.pose.position.x = x
    odom.pose.pose.position.y = y
    fill_yaw(odom.pose.pose.orientation, yaw)
    return odom


def test_p_rejected_without_imu_and_encoder_does_not_publish_blackout():
    node = make_node(imu_topic='')
    node._rf2o(rf2o_message())

    node._blackout_request(Empty())

    assert node.state is Mode11State.NORMAL
    assert not node.fallback.active
    assert not node.state_publisher.messages
    assert not node.odom_publisher.messages
    assert 'Encoder /wheel_ticks' in node.logger.errors[-1]
    assert 'IMU topic configuration' in node.logger.errors[-1]


def test_virtual_step_counts_require_explicit_opt_in():
    node = make_node(allow_virtual_step_counts=False)
    node._rf2o(rf2o_message())
    node._encoder(Int64MultiArray(data=[100, -100]))
    node._encoder(Int64MultiArray(data=[100, -100]))
    imu = Imu()
    imu.header.stamp = Time(sec=100)
    node._imu(imu)
    imu.header.stamp = Time(sec=100, nanosec=50_000_000)
    node._imu(imu)

    node._blackout_request(Empty())

    assert node.state is Mode11State.NORMAL
    assert not node.fallback.active
    assert 'physical Encoder' in node.logger.errors[-1]


def test_p_seeds_last_rf2o_pose_and_stops_scan_and_rf2o_selection():
    node = make_node()
    rf2o = rf2o_message()
    node._rf2o(rf2o)
    node._encoder(Int64MultiArray(data=[100, -100]))
    node._encoder(Int64MultiArray(data=[100, -100]))
    imu = Imu()
    imu.header.stamp = Time(sec=100)
    imu.angular_velocity.z = 0.5
    node._imu(imu)
    imu.header.stamp = Time(sec=100, nanosec=50_000_000)
    node._imu(imu)
    scan = LaserScan()
    scan.header.stamp = Time(sec=100)
    node._scan(scan)
    assert node.scan_publisher.messages == [scan]

    node._blackout_request(Empty())

    assert node.state is Mode11State.BLACKOUT
    assert node.fallback.active
    assert node.fallback.pose.x == 4.2
    assert node.fallback.pose.y == 1.8
    assert math.isclose(node.fallback.pose.yaw, math.pi / 6, abs_tol=1e-12)
    assert node.state_publisher.messages[-1].data == 'BLACKOUT'
    assert 'LiDAR 작동 멈춤 - Encoder + IMU 위치추정 전환' in node.logger.warnings[-1]

    selected = node.odom_publisher.messages[-1]
    assert len(node.body_publisher.messages[-1].markers) == 2
    assert selected.header.frame_id == 'odom'
    assert selected.child_frame_id == 'base_link'
    assert selected.pose.pose.position.x == 4.2
    assert selected.pose.pose.position.y == 1.8
    assert math.isclose(yaw_from_quaternion(selected.pose.pose.orientation),
                        math.pi / 6, abs_tol=1e-12)
    assert [(tf.header.frame_id, tf.child_frame_id)
            for tf in node.tf_broadcaster.transforms] == [
                ('odom', 'base_link'), ('map', 'odom')]

    # Even valid, fresh LiDAR input and a different RF2O pose are ignored.
    node._scan(scan)
    node._rf2o(rf2o_message(x=99.0, y=99.0))
    node._publish()
    assert node.scan_publisher.messages == [scan]
    assert node.rf2o_pose.x == 4.2
    assert node.odom_publisher.messages[-1].pose.pose.position.x == 4.2


def test_p_rejects_imu_without_angular_velocity_and_bad_sample_interval():
    node = make_node()
    node._rf2o(rf2o_message())
    node._encoder(Int64MultiArray(data=[100, -100]))
    node._encoder(Int64MultiArray(data=[100, -100]))
    imu = Imu()
    imu.header.stamp = Time(sec=100)
    node._imu(imu)
    imu.header.stamp = Time(sec=100, nanosec=50_000_000)
    imu.angular_velocity_covariance[0] = -1.0
    node._imu(imu)
    node._blackout_request(Empty())
    assert node.state is Mode11State.NORMAL
    assert 'IMU /tested/imu' in node.logger.errors[-1]

    imu.angular_velocity_covariance[0] = 0.0
    imu.header.stamp = Time(sec=100, nanosec=400_000_000)
    node.clock.seconds = 100.4
    node._imu(imu)
    node._blackout_request(Empty())
    assert node.state is Mode11State.NORMAL
    assert 'IMU /tested/imu' in node.logger.errors[-1]


def test_normal_stale_rf2o_does_not_receive_fresh_selected_timestamps():
    node = make_node()
    node._rf2o(rf2o_message())
    node._publish()
    assert len(node.odom_publisher.messages) == 1
    node.clock.seconds = 101.0
    node._publish()
    assert len(node.odom_publisher.messages) == 1
    assert node.rf2o_stale_warning



def test_mapless_mode_can_enter_blackout_without_amcl_transform():
    node = make_node()
    node.encoder_topic = '/wheel_encoder_ticks'
    node.require_map_alignment = False
    node.map_to_odom = None
    node._rf2o(rf2o_message())
    node._encoder(Int64MultiArray(data=[100, 100]))
    node._encoder(Int64MultiArray(data=[100, 100]))
    imu = Imu()
    imu.header.stamp = Time(sec=100)
    node._imu(imu)
    imu.header.stamp = Time(sec=100, nanosec=50_000_000)
    node._imu(imu)

    node._blackout_request(Empty())

    assert node.state is Mode11State.BLACKOUT
    assert node.fallback.pose == Pose2D(4.2, 1.8, math.pi / 6)
    assert [(tf.header.frame_id, tf.child_frame_id)
            for tf in node.tf_broadcaster.transforms] == [
                ('odom', 'base_link')]
