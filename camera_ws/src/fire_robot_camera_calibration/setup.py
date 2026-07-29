from glob import glob
import os

from setuptools import find_packages, setup


package_name = 'fire_robot_camera_calibration'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name],
        ),
        ('share/' + package_name, ['package.xml', 'README.md']),
        (
            os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py'),
        ),
        (
            os.path.join('share', package_name, 'config'),
            glob('config/*.yaml'),
        ),
    ],
    install_requires=['setuptools'],
    tests_require=['pytest'],
    zip_safe=True,
    maintainer='kai',
    maintainer_email='sunwoo050223@naver.com',
    description=(
        'Intrinsic camera and 2D LiDAR-to-camera extrinsic calibration tools '
        'for Ubuntu 24.04 and ROS 2 Jazzy.'
    ),
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'calibrate_fisheye = '
            'fire_robot_camera_calibration.calibrate_fisheye:main',
            'external_tuner = '
            'fire_robot_camera_calibration.external_tuner_node:main',
            'guided_capture = '
            'fire_robot_camera_calibration.guided_capture_node:main',
            'rectify_camera = '
            'fire_robot_camera_calibration.rectify_node:main',
            'static_tf_publisher = '
            'fire_robot_camera_calibration.static_tf_publisher:main',
            'tf_overlay = '
            'fire_robot_camera_calibration.tf_overlay_node:main',
        ],
    },
)
