from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument,IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration as L
from launch_ros.actions import Node

def generate_launch_description():
    s=get_package_share_directory('inno_robot_bringup')
    d={'start_lidar':'true','serial_port':'/dev/ttyUSB0','serial_baudrate':'460800','scan_topic':'/scan','use_rf2o':'true','use_rviz':'true','use_path':'true','use_sim_time':'false','map_frame':'map','odom_frame':'odom','base_frame':'base_link','laser_frame':'laser','laser_x':'0.0','laser_y':'0.0','laser_z':'0.60','laser_roll':'0.0','laser_pitch':'0.0','laser_yaw':'0.0','slam_params_file':s+'/config/slam_toolbox_online_async.yaml','rf2o_params_file':s+'/config/rf2o.yaml','rviz_config_file':s+'/rviz/inno_slam.rviz'}
    args=[DeclareLaunchArgument(k,default_value=v) for k,v in d.items()]
    common={k:L(k) for k in ('scan_topic','base_frame','laser_frame','laser_x','laser_y','laser_z','laser_roll','laser_pitch','laser_yaw')}
    lidar=IncludeLaunchDescription(PythonLaunchDescriptionSource(s+'/launch/lidar_only.launch.py'),launch_arguments={**common,'start_lidar':L('start_lidar'),'serial_port':L('serial_port'),'serial_baudrate':L('serial_baudrate'),'publish_static_tf':'false'}.items())
    slam=IncludeLaunchDescription(PythonLaunchDescriptionSource(s+'/launch/slam_only.launch.py'),launch_arguments={**common,'use_rf2o':L('use_rf2o'),'use_path':L('use_path'),'use_sim_time':L('use_sim_time'),'map_frame':L('map_frame'),'odom_frame':L('odom_frame'),'slam_params_file':L('slam_params_file'),'rf2o_params_file':L('rf2o_params_file')}.items())
    rviz=Node(package='rviz2',executable='rviz2',name='rviz2',condition=IfCondition(L('use_rviz')),arguments=['-d',L('rviz_config_file')],parameters=[{'use_sim_time':L('use_sim_time')}],output='screen')
    return LaunchDescription(args+[lidar,slam,rviz])
