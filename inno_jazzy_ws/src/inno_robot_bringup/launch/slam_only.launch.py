from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument,IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration as L
from launch_ros.actions import Node

def generate_launch_description():
    share=get_package_share_directory('inno_robot_bringup'); slam_share=get_package_share_directory('slam_toolbox')
    d={'slam_params_file':share+'/config/slam_toolbox_online_async.yaml','rf2o_params_file':share+'/config/rf2o.yaml','scan_topic':'/scan','use_rf2o':'true','use_path':'true','use_sim_time':'false','map_frame':'map','odom_frame':'odom','base_frame':'base_link','laser_frame':'laser','laser_x':'0.0','laser_y':'0.0','laser_z':'0.60','laser_roll':'0.0','laser_pitch':'0.0','laser_yaw':'0.0'}
    args=[DeclareLaunchArgument(k,default_value=v) for k,v in d.items()]
    tf=Node(package='tf2_ros',executable='static_transform_publisher',name='base_to_laser_tf',arguments=['--x',L('laser_x'),'--y',L('laser_y'),'--z',L('laser_z'),'--roll',L('laser_roll'),'--pitch',L('laser_pitch'),'--yaw',L('laser_yaw'),'--frame-id',L('base_frame'),'--child-frame-id',L('laser_frame')])
    rf2o=Node(package='rf2o_laser_odometry',executable='rf2o_laser_odometry_node',name='rf2o_laser_odometry',condition=IfCondition(L('use_rf2o')),output='screen',parameters=[L('rf2o_params_file'),{'laser_scan_topic':L('scan_topic'),'odom_frame_id':L('odom_frame'),'base_frame_id':L('base_frame'),'use_sim_time':L('use_sim_time')}])
    path=Node(package='inno_robot_bringup',executable='odom_to_path',name='odom_to_path',condition=IfCondition(L('use_path')),output='screen',parameters=[{'odom_topic':'/odom_rf2o','path_topic':'/rf2o_path','frame_id':L('odom_frame'),'max_points':5000,'use_sim_time':L('use_sim_time')}])
    slam=IncludeLaunchDescription(PythonLaunchDescriptionSource(slam_share+'/launch/online_async_launch.py'),launch_arguments={'use_sim_time':L('use_sim_time'),'slam_params_file':L('slam_params_file')}.items())
    return LaunchDescription(args+[tf,rf2o,path,slam])
