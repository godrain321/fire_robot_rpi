"""Mode 6: thermal-camera-only bench test.

Keyboard driving (drive mode 1) + LiDAR/AMCL localization + the MLX90640 thermal
stack, with the thermal cost grid shown in RViz. No hazard belief, no gas, no
planner, no replanning -- this profile exists purely to look at
``/thermal_cost_grid`` while nudging the robot around by keyboard.
"""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration as L
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from inno_robot_bringup.project_paths import project_path


def generate_launch_description():
    bringup = get_package_share_directory("inno_robot_bringup")
    thermal = get_package_share_directory("inno_thermal")
    drive = get_package_share_directory("inno_drive_bridge")
    autonav = get_package_share_directory("inno_autonav")
    drive_params = drive + "/config/drive_params.yaml"

    args = [
        DeclareLaunchArgument("use_serial", default_value="true"),
        DeclareLaunchArgument("use_rviz", default_value="true"),
        DeclareLaunchArgument("start_lidar", default_value="true"),
        DeclareLaunchArgument("esp32_port", default_value="/dev/ttyUSB0"),
        DeclareLaunchArgument("lidar_port", default_value="/dev/ttyUSB1"),
        DeclareLaunchArgument(
            "map_yaml", default_value=project_path("maps", "inno_map_raw.yaml")
        ),
        DeclareLaunchArgument(
            "planning_map_yaml",
            default_value=project_path("maps", "inno_map_nav.yaml"),
        ),
        DeclareLaunchArgument("set_initial_pose", default_value="false"),
        DeclareLaunchArgument("initial_pose_x", default_value="0.0"),
        DeclareLaunchArgument("initial_pose_y", default_value="0.0"),
        DeclareLaunchArgument("initial_pose_yaw", default_value="0.0"),
        DeclareLaunchArgument("linear_speed", default_value="0.08"),
        DeclareLaunchArgument("angular_speed", default_value="0.35"),
        # base_link -> thermal_camera_link mounting offset (same names as Mode 5).
        DeclareLaunchArgument("thermal_x", default_value="0.10"),
        DeclareLaunchArgument("thermal_y", default_value="0.0"),
        DeclareLaunchArgument("thermal_z", default_value="0.20"),
        DeclareLaunchArgument("thermal_roll", default_value="0.0"),
        DeclareLaunchArgument("thermal_pitch", default_value="0.0"),
        DeclareLaunchArgument("thermal_yaw", default_value="0.0"),
    ]

    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            bringup + "/launch/lidar_amcl_localization.launch.py"
        ),
        launch_arguments={
            "start_lidar": L("start_lidar"),
            "serial_port": L("lidar_port"),
            "map_yaml": L("map_yaml"),
            "set_initial_pose": L("set_initial_pose"),
            "initial_pose_x": L("initial_pose_x"),
            "initial_pose_y": L("initial_pose_y"),
            "initial_pose_yaw": L("initial_pose_yaw"),
        }.items(),
    )

    # thermal_cost_layer needs the static planning grid as its geometry template.
    planning_grid = Node(
        package="inno_autonav", executable="planning_grid_publisher",
        name="planning_grid_publisher", output="screen",
        parameters=[
            autonav + "/config/autonav_params.yaml",
            {"map_yaml": L("planning_map_yaml")},
        ],
    )

    base_to_thermal_tf = Node(
        package="tf2_ros", executable="static_transform_publisher",
        name="base_to_thermal_camera_tf",
        arguments=[
            "--x", L("thermal_x"), "--y", L("thermal_y"), "--z", L("thermal_z"),
            "--roll", L("thermal_roll"), "--pitch", L("thermal_pitch"),
            "--yaw", L("thermal_yaw"),
            "--frame-id", "base_link", "--child-frame-id", "thermal_camera_link",
        ],
    )

    thermal_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            thermal + "/launch/thermal_sensor.launch.py"
        ),
        launch_arguments={"enable_cost_layer": "true"}.items(),
    )

    keyboard = Node(
        package="inno_drive_bridge", executable="keyboard_cmdvel_demo",
        name="keyboard_cmdvel_demo", output="screen", emulate_tty=True,
        parameters=[
            drive_params,
            {
                "linear_speed": ParameterValue(L("linear_speed"), value_type=float),
                "angular_speed": ParameterValue(L("angular_speed"), value_type=float),
            },
        ],
    )
    mux = Node(
        package="inno_drive_bridge", executable="cmd_vel_mode_mux",
        name="cmd_vel_mode_mux", output="log", parameters=[drive_params],
    )
    serial = Node(
        package="inno_drive_bridge", executable="cmdvel_to_esp32_serial",
        name="cmdvel_to_esp32_serial", output="log",
        parameters=[drive_params, {"serial_port": L("esp32_port")}],
        condition=IfCondition(L("use_serial")),
    )
    step_odom = Node(
        package="inno_drive_bridge", executable="step_count_to_odom",
        name="step_count_to_odom", output="log", parameters=[drive_params],
        condition=IfCondition(L("use_serial")),
    )

    rviz = Node(
        package="rviz2", executable="rviz2", name="mode6_thermal_rviz",
        arguments=["-d", bringup + "/rviz/mode6_thermal.rviz"],
        output="screen", condition=IfCondition(L("use_rviz")),
    )

    return LaunchDescription(args + [
        localization, planning_grid, base_to_thermal_tf, thermal_stack,
        keyboard, mux, serial, step_odom, rviz,
    ])
