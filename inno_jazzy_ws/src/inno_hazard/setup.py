from glob import glob
import os

from setuptools import find_packages, setup


package_name = 'inno_hazard'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='gosunwoo',
    maintainer_email='gosunwoo@example.com',
    description='Sensor-only hazard belief and exact float planning costs.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'hazard_belief_node = inno_hazard.hazard_belief_node:main',
            'planning_grid_hazard_merge = '
            'inno_hazard.planning_grid_hazard_merge:main',
        ],
    },
)
