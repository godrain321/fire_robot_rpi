from glob import glob
from setuptools import find_packages, setup


package_name = "inno_camera_tools"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="seeno04",
    maintainer_email="seeno04@example.com",
    description="Camera Module 3 launch and calibration helpers for the INNO robot.",
    license="Apache-2.0",
)
