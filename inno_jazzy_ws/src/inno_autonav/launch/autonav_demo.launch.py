"""Launch the custom autonomous navigation pipeline without localization."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from inno_autonav.project_paths import project_path


def generate_launch_description() -> LaunchDescription:
    package_share = get_package_share_directory('inno_autonav')
    hazard_share = get_package_share_directory('inno_hazard')
    drive_share = get_package_share_directory('inno_drive_bridge')
    config_file = os.path.join(package_share, 'config', 'autonav_params.yaml')
    drive_config = os.path.join(drive_share, 'config', 'drive_params.yaml')
    use_serial = LaunchConfiguration('use_serial')
    serial_port = LaunchConfiguration('serial_port')
    use_wheel_odom_tf = LaunchConfiguration('use_wheel_odom_tf')
    map_yaml = LaunchConfiguration('map_yaml')
    semantic_yaml = LaunchConfiguration('semantic_yaml')
    use_dynamic_obstacles = LaunchConfiguration('use_dynamic_obstacles')
    person_inspection_enabled = LaunchConfiguration('person_inspection_enabled')
    max_linear_speed = LaunchConfiguration('max_linear_speed')
    max_angular_speed = LaunchConfiguration('max_angular_speed')
    require_thermal_grid = LaunchConfiguration('require_thermal_grid')
    require_thermal_active = LaunchConfiguration('require_thermal_active')
    waypoint_file = LaunchConfiguration('waypoint_file')
    hazard_belief_enabled = LaunchConfiguration('hazard_belief_enabled')
    hazard_thermal_enabled = LaunchConfiguration('hazard_thermal_enabled')
    temperature_cost_scale_max_c = LaunchConfiguration(
        'temperature_cost_scale_max_c'
    )
    temperature_blocked_c = LaunchConfiguration('temperature_blocked_c')
    hazard_co_enabled = LaunchConfiguration('hazard_co_enabled')
    gas_input_mode = LaunchConfiguration('gas_input_mode')
    gas_safe_adc = LaunchConfiguration('gas_safe_adc')
    gas_blocked_adc = LaunchConfiguration('gas_blocked_adc')
    exit_evaluator_enabled = LaunchConfiguration('exit_evaluator_enabled')
    evacuation_manager_enabled = LaunchConfiguration('evacuation_manager_enabled')
    evacuation_activate_route = LaunchConfiguration(
        'evacuation_activate_selected_route'
    )
    event_replanning_enabled = LaunchConfiguration('event_replanning_enabled')
    astar_periodic_replanning_enabled = LaunchConfiguration(
        'astar_periodic_replanning_enabled'
    )
    exit_switching_enabled = LaunchConfiguration('exit_switching_enabled')
    waypoint_planning_enabled = LaunchConfiguration('waypoint_planning_enabled')
    waypoint_accept_direct_goal = LaunchConfiguration('waypoint_accept_direct_goal')
    waypoint_planning_grid_topic = LaunchConfiguration(
        'waypoint_planning_grid_topic'
    )
    astar_path_output_topic = LaunchConfiguration('astar_path_output_topic')
    astar_accept_goal_pose = LaunchConfiguration('astar_accept_goal_pose')
    mode3_standoff_distance = LaunchConfiguration(
        'mode3_standoff_distance_m'
    )
    mode3_publish_canonical_plan = LaunchConfiguration(
        'mode3_publish_canonical_plan'
    )
    mode4_standoff_distance = LaunchConfiguration(
        'mode4_standoff_distance_m'
    )
    mode4_publish_canonical_plan = LaunchConfiguration(
        'mode4_publish_canonical_plan'
    )
    mode4_minimum_confidence = LaunchConfiguration(
        'mode4_minimum_confidence'
    )
    require_localization_ready = LaunchConfiguration(
        'require_localization_ready'
    )
    hazard_config = os.path.join(hazard_share, 'config', 'hazard_params.yaml')

    return LaunchDescription(
        [
            DeclareLaunchArgument('use_serial', default_value='false'),
            DeclareLaunchArgument('serial_port', default_value='/dev/ttyUSB0'),
            DeclareLaunchArgument('use_wheel_odom_tf', default_value='false'),
            DeclareLaunchArgument('use_dynamic_obstacles', default_value='false'),
            DeclareLaunchArgument('person_inspection_enabled', default_value='true'),
            DeclareLaunchArgument('max_linear_speed', default_value='0.06'),
            DeclareLaunchArgument('max_angular_speed', default_value='0.45'),
            DeclareLaunchArgument('require_thermal_grid', default_value='true'),
            DeclareLaunchArgument('require_thermal_active', default_value='true'),
            DeclareLaunchArgument(
                'waypoint_file',
                default_value=project_path(
                    'maps', 'waypoint_queue_latest.yaml'
                ),
            ),
            DeclareLaunchArgument(
                'hazard_belief_enabled', default_value='false'
            ),
            DeclareLaunchArgument(
                'hazard_thermal_enabled', default_value='true'
            ),
            DeclareLaunchArgument(
                'temperature_cost_scale_max_c', default_value='60.0'
            ),
            DeclareLaunchArgument(
                'temperature_blocked_c', default_value='60.0'
            ),
            DeclareLaunchArgument(
                'hazard_co_enabled', default_value='false'
            ),
            DeclareLaunchArgument(
                'gas_input_mode', default_value='legacy_ppm'
            ),
            DeclareLaunchArgument('gas_safe_adc', default_value='0.0'),
            DeclareLaunchArgument('gas_blocked_adc', default_value='4096.0'),
            DeclareLaunchArgument(
                'exit_evaluator_enabled', default_value='false'
            ),
            DeclareLaunchArgument(
                'evacuation_manager_enabled', default_value='false'
            ),
            DeclareLaunchArgument(
                'evacuation_activate_selected_route', default_value='false'
            ),
            DeclareLaunchArgument(
                'event_replanning_enabled', default_value='false'
            ),
            # Stage 7 is event/dirty driven by default. This remains an explicit
            # compatibility switch for field diagnostics that require legacy
            # unconditional periodic A*.
            DeclareLaunchArgument(
                'astar_periodic_replanning_enabled',
                default_value='false',
            ),
            DeclareLaunchArgument(
                'exit_switching_enabled', default_value='false'
            ),
            DeclareLaunchArgument(
                'waypoint_planning_enabled', default_value='false'
            ),
            DeclareLaunchArgument(
                'waypoint_accept_direct_goal', default_value='false'
            ),
            # Auto-connected to waypoint_planning_enabled (Stage 8-8): when the
            # waypoint pipeline is on, astar_replanner's output moves off
            # /planned_path so PathSelector becomes the single owner of it,
            # matching the final-ownership diagram. When off, astar_replanner
            # keeps publishing /planned_path directly -- Stage 1-7 unchanged.
            DeclareLaunchArgument(
                'astar_path_output_topic',
                default_value=PythonExpression([
                    "'/astar_path' if '", waypoint_planning_enabled,
                    "' == 'true' else '/planned_path'"
                ]),
            ),
            # Stage 5: when the gas sensor is on, the waypoint planner reads the
            # gas-inclusive grid (astar_replanner already reads /hazard/final_cost
            # directly). Off -> the untouched /planning_grid, Stage 1-4 unchanged.
            DeclareLaunchArgument(
                'waypoint_planning_grid_topic',
                default_value=PythonExpression([
                    "'/planning_grid_hazard' if '", hazard_co_enabled,
                    "' == 'true' else '/planning_grid'"
                ]),
            ),
            DeclareLaunchArgument('astar_accept_goal_pose', default_value='true'),
            DeclareLaunchArgument(
                'mode3_standoff_distance_m', default_value='2.0'
            ),
            DeclareLaunchArgument(
                'mode3_publish_canonical_plan', default_value='false'
            ),
            DeclareLaunchArgument(
                'mode4_standoff_distance_m', default_value='1.5'
            ),
            DeclareLaunchArgument(
                'mode4_publish_canonical_plan', default_value='false'
            ),
            DeclareLaunchArgument(
                'mode4_minimum_confidence', default_value='0.40'
            ),
            DeclareLaunchArgument(
                'require_localization_ready', default_value='false'
            ),
            DeclareLaunchArgument(
                'map_yaml',
                default_value=project_path('maps', 'inno_map_nav.yaml'),
            ),
            DeclareLaunchArgument(
                'semantic_yaml',
                default_value=project_path(
                    'inno_jazzy_ws', 'src', 'inno_autonav', 'config',
                    'semantic_points.yaml',
                ),
            ),
            Node(
                package='inno_hazard',
                executable='hazard_belief_node',
                name='hazard_belief_node',
                parameters=[hazard_config, {
                    'thermal_enabled': ParameterValue(
                        hazard_thermal_enabled, value_type=bool
                    ),
                    'temperature_cost_scale_max_c': ParameterValue(
                        temperature_cost_scale_max_c, value_type=float
                    ),
                    'temperature_blocked_c': ParameterValue(
                        temperature_blocked_c, value_type=float
                    ),
                    'co_enabled': ParameterValue(
                        hazard_co_enabled, value_type=bool
                    ),
                    'gas_input_mode': ParameterValue(
                        gas_input_mode, value_type=str
                    ),
                    'gas_safe_adc': ParameterValue(
                        gas_safe_adc, value_type=float
                    ),
                    'gas_blocked_adc': ParameterValue(
                        gas_blocked_adc, value_type=float
                    ),
                }],
                output='screen',
                condition=IfCondition(hazard_belief_enabled),
            ),
            # Stage 5: fold the gas cost overlay into a planner-consumable grid
            # for the waypoint planner. A* is unaffected (it reads
            # /hazard/final_cost directly). Only runs when the gas sensor is on.
            Node(
                package='inno_hazard',
                executable='planning_grid_hazard_merge',
                name='planning_grid_hazard_merge',
                output='screen',
                condition=IfCondition(hazard_co_enabled),
            ),
            Node(
                package='inno_autonav',
                executable='planning_grid_publisher',
                name='planning_grid_publisher',
                parameters=[config_file, {'map_yaml': map_yaml}],
                output='screen',
            ),
            Node(
                package='inno_autonav',
                executable='dynamic_obstacle_layer',
                name='dynamic_obstacle_layer',
                parameters=[config_file],
                output='screen',
                condition=IfCondition(use_dynamic_obstacles),
            ),
            Node(
                package='inno_autonav',
                executable='astar_replanner',
                name='astar_replanner',
                parameters=[
                    config_file,
                    {
                        'require_thermal_grid': ParameterValue(
                            require_thermal_grid, value_type=bool
                        ),
                        'require_thermal_active': ParameterValue(
                            require_thermal_active, value_type=bool
                        ),
                        'reference_waypoint_file': waypoint_file,
                        'hazard_belief_enabled': ParameterValue(
                            hazard_belief_enabled, value_type=bool
                        ),
                        'periodic_replanning_enabled': ParameterValue(
                            astar_periodic_replanning_enabled, value_type=bool
                        ),
                        'path_output_topic': astar_path_output_topic,
                        'accept_goal_pose': ParameterValue(
                            astar_accept_goal_pose, value_type=bool
                        ),
                    },
                ],
                output='screen',
            ),
            Node(
                package='inno_autonav',
                executable='exit_evaluator_node',
                name='exit_evaluator_node',
                parameters=[
                    config_file,
                    {
                        'exit_registry_file': semantic_yaml,
                        'reference_waypoint_file': waypoint_file,
                    },
                ],
                output='screen',
                condition=IfCondition(exit_evaluator_enabled),
            ),
            Node(
                package='inno_autonav',
                executable='evacuation_manager_node',
                name='evacuation_manager_node',
                parameters=[
                    config_file,
                    {
                        'enabled': ParameterValue(
                            evacuation_manager_enabled, value_type=bool
                        ),
                        'activate_selected_route': ParameterValue(
                            evacuation_activate_route, value_type=bool
                        ),
                    },
                ],
                output='screen',
                condition=IfCondition(evacuation_manager_enabled),
            ),
            Node(
                package='inno_autonav',
                executable='replan_supervisor_node',
                name='replan_supervisor_node',
                parameters=[
                    config_file,
                    {
                        'enabled': ParameterValue(
                            event_replanning_enabled, value_type=bool
                        ),
                        'waypoint_planning_enabled': ParameterValue(
                            waypoint_planning_enabled, value_type=bool
                        ),
                    },
                ],
                output='screen',
                condition=IfCondition(event_replanning_enabled),
            ),
            Node(
                package='inno_autonav',
                executable='exit_switching_node',
                name='exit_switching_node',
                parameters=[
                    config_file,
                    {
                        'exit_registry_file': semantic_yaml,
                        'enabled': ParameterValue(
                            exit_switching_enabled, value_type=bool
                        ),
                    },
                ],
                output='screen',
                condition=IfCondition(exit_switching_enabled),
            ),
            Node(
                package='inno_autonav',
                executable='waypoint_planner_node',
                name='waypoint_planner_node',
                parameters=[
                    config_file,
                    {
                        'waypoint_file': waypoint_file,
                        'enabled': ParameterValue(
                            waypoint_planning_enabled, value_type=bool
                        ),
                        'accept_direct_goal': ParameterValue(
                            waypoint_accept_direct_goal, value_type=bool
                        ),
                        'planning_grid_topic': waypoint_planning_grid_topic,
                    },
                ],
                output='screen',
                condition=IfCondition(waypoint_planning_enabled),
            ),
            Node(
                package='inno_autonav',
                executable='path_selector_node',
                name='path_selector_node',
                parameters=[config_file],
                output='screen',
                condition=IfCondition(waypoint_planning_enabled),
            ),
            Node(
                package='inno_autonav',
                executable='mode3_inspector',
                name='mode3_inspector',
                parameters=[
                    config_file,
                    {
                        'standoff_distance_m': ParameterValue(
                            mode3_standoff_distance, value_type=float
                        ),
                        'publish_canonical_plan': ParameterValue(
                            mode3_publish_canonical_plan, value_type=bool
                        ),
                    },
                ],
                output='screen',
                emulate_tty=True,
                condition=IfCondition(PythonExpression([
                    "'", use_dynamic_obstacles, "' == 'true' and '",
                    person_inspection_enabled, "' == 'true'",
                ])),
            ),
            Node(
                package='inno_autonav',
                executable='mode4_inspector',
                name='mode4_inspector',
                parameters=[
                    config_file,
                    {
                        'standoff_distance_m': ParameterValue(
                            mode4_standoff_distance, value_type=float
                        ),
                        'publish_canonical_plan': ParameterValue(
                            mode4_publish_canonical_plan, value_type=bool
                        ),
                        'minimum_confidence': ParameterValue(
                            mode4_minimum_confidence, value_type=float
                        ),
                    },
                ],
                output='screen',
                emulate_tty=True,
                condition=IfCondition(PythonExpression([
                    "'", use_dynamic_obstacles, "' == 'true' and '",
                    person_inspection_enabled, "' == 'true'",
                ])),
            ),
            Node(
                package='inno_autonav',
                executable='skid_path_follower',
                name='skid_path_follower',
                parameters=[
                    config_file,
                    {
                        'max_linear_speed': ParameterValue(
                            max_linear_speed, value_type=float
                        ),
                        'max_angular_speed': ParameterValue(
                            max_angular_speed, value_type=float
                        ),
                        'require_localization_ready': ParameterValue(
                            require_localization_ready, value_type=bool
                        ),
                    },
                ],
                output='screen',
            ),
            Node(
                package='inno_autonav',
                executable='mission_commander',
                name='mission_commander',
                parameters=[config_file, {'semantic_yaml': semantic_yaml}],
                output='screen',
                condition=UnlessCondition(evacuation_manager_enabled),
            ),
            Node(
                package='inno_drive_bridge',
                executable='cmdvel_to_esp32_serial',
                name='cmdvel_to_esp32_serial',
                parameters=[drive_config, {'serial_port': serial_port}],
                output='screen',
                emulate_tty=True,
                condition=IfCondition(use_serial),
            ),
            Node(
                package='inno_drive_bridge',
                executable='step_count_to_odom',
                name='step_count_to_odom',
                parameters=[
                    drive_config,
                    {
                        'publish_tf': True,
                        'odom_frame': 'odom',
                        'base_frame': 'base_link',
                    },
                ],
                output='screen',
                condition=IfCondition(use_wheel_odom_tf),
            ),
        ]
    )
