"""Launch the MLX90640 sensor node with the packaged default parameters."""

from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from pathlib import Path


def generate_launch_description():
    parameters = (
        Path(get_package_share_directory("inno_thermal"))
        / "config"
        / "thermal_params.yaml"
    )
    enable_cost_layer = LaunchConfiguration("enable_cost_layer")
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "enable_cost_layer",
                default_value="true",
                description="Start the map-frame thermal cost grid node",
            ),
            DeclareLaunchArgument(
                "temperature_cost_scale_max_c", default_value="60.0"
            ),
            DeclareLaunchArgument(
                "blocked_temperature_c", default_value="60.0"
            ),
            Node(
                package="inno_thermal",
                executable="mlx90640_sensor_node",
                name="mlx90640_sensor_node",
                output="screen",
                parameters=[str(parameters)],
            ),
            Node(
                package="inno_thermal",
                executable="thermal_cost_layer",
                name="thermal_cost_layer",
                output="screen",
                parameters=[str(parameters), {
                    "temperature_cost_scale_max_c": ParameterValue(
                        LaunchConfiguration("temperature_cost_scale_max_c"),
                        value_type=float,
                    ),
                    "blocked_temperature_c": ParameterValue(
                        LaunchConfiguration("blocked_temperature_c"),
                        value_type=float,
                    ),
                }],
                condition=IfCondition(enable_cost_layer),
            ),
        ]
    )
