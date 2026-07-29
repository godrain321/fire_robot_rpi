from glob import glob
import os

from setuptools import find_packages, setup


package_name = 'inno_map_tools'


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
    description='No-go mask and planning map generation tools.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'save_clicked_points = inno_map_tools.save_clicked_points:main',
            'build_no_go_mask = inno_map_tools.build_no_go_mask:main',
        ],
    },
)
