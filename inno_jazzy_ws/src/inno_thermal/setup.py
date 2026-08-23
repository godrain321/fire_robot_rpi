from glob import glob
import os

from setuptools import find_packages, setup


package_name = "inno_thermal"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        ("share/" + package_name, ["package.xml", "README.md"]),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "native"), glob("native/*.so")),
        (os.path.join("share", package_name, "scripts"), glob("scripts/*.sh")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="seeno04",
    maintainer_email="seeno04@example.com",
    description="MLX90640 raw thermal sensing and short-arc projection for ROS 2.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "mlx90640_sensor_node = inno_thermal.mlx90640_sensor_node:main",
            "thermal_cost_layer = inno_thermal.thermal_cost_layer:main",
        ],
    },
)
