from glob import glob
import os

from setuptools import find_packages, setup


package_name = 'inno_semantic_nav'


setup(
    name=package_name,
    version='0.2.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kai',
    maintainer_email='kai@todo.todo',
    description='Semantic waypoint capture and navigation tools for ROS 2 Jazzy.',
    license='Apache-2.0',
    python_requires='>=3.10',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'capture_named_pose = inno_semantic_nav.capture_named_pose:main',
            'capture_landmark = inno_semantic_nav.capture_landmark:main',
            'semantic_marker_node = inno_semantic_nav.semantic_marker_node:main',
            'go = inno_semantic_nav.go_named_pose:main',
        ],
    },
)
