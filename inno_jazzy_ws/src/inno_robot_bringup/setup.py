from glob import glob
from setuptools import find_packages, setup


package_name = "inno_robot_bringup"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
        ("share/" + package_name + "/rviz", glob("rviz/*.rviz")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="seeno04",
    maintainer_email="seeno04@example.com",
    description="Bringup for Camera Module 3, RPLIDAR C1, and RF2O odometry.",
    license="Apache-2.0",
    entry_points={"console_scripts": [
        "odom_to_path = inno_robot_bringup.odom_to_path_node:main",
        "slam_keyboard_runner = inno_robot_bringup.slam_keyboard_runner:main",
    ]},
)
