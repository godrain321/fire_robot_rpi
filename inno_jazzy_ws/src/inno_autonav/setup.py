from glob import glob
import os

from setuptools import find_packages, setup


package_name = 'inno_autonav'


setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='gosunwoo',
    maintainer_email='gosunwoo@example.com',
    description='A* replanning and skid-drive path following.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mission_commander = inno_autonav.mission_commander:main',
            'go_to = inno_autonav.go_to:main',
            'planning_grid_publisher = inno_autonav.planning_grid_publisher:main',
            'dynamic_obstacle_layer = inno_autonav.dynamic_obstacle_layer:main',
            'astar_replanner = inno_autonav.astar_replanner:main',
            'skid_path_follower = inno_autonav.skid_path_follower:main',
            'waypoint_queue = inno_autonav.waypoint_queue:main',
        ],
    },
)
