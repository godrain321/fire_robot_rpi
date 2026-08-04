from glob import glob
import os

from setuptools import find_packages, setup


package_name = 'inno_drive_bridge'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='seeno04',
    maintainer_email='seeno04@example.com',
    description='USB serial drive bridge and step-count odometry for the fire robot.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'keyboard_cmdvel_demo = inno_drive_bridge.keyboard_cmdvel_demo:main',
            'cmdvel_to_esp32_serial = inno_drive_bridge.cmdvel_to_esp32_serial:main',
            'step_count_to_odom = inno_drive_bridge.step_count_to_odom:main',
            'cmd_vel_mode_mux = inno_drive_bridge.cmd_vel_mode_mux:main',
        ],
    },
)
