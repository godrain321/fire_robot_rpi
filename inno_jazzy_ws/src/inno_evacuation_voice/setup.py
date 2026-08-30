from glob import glob
import os

from setuptools import find_packages, setup


package_name = 'inno_evacuation_voice'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'audio'), glob('audio/*.wav')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='gosunwoo',
    maintainer_email='gosunwoo@example.com',
    description='Offline periodic evacuation voice guidance.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'periodic_evacuation_voice_node = '
            'inno_evacuation_voice.periodic_evacuation_voice_node:main',
        ],
    },
)
