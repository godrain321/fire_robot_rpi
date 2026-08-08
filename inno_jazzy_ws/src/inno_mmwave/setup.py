from glob import glob
import os

from setuptools import find_packages, setup


package_name = 'inno_mmwave'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name],
        ),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='seeno04',
    maintainer_email='seeno04@example.com',
    description='DFRobot C4001 mmWave sensing and visualization for the fire robot.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'c4001_node = inno_mmwave.c4001_node:main',
            'mmwave_mobility = inno_mmwave.mobility_node:main',
            'mmwave_gui = inno_mmwave.mmwave_gui:main',
            'mmwave_status_console = inno_mmwave.status_console:main',
            'mmwave_presence_led = inno_mmwave.presence_led:main',
        ],
    },
)
