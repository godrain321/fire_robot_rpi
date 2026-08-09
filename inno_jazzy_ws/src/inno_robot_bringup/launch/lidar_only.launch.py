from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration as L
from launch_ros.actions import Node

def generate_launch_description():
    defaults={'start_lidar':'true','serial_port':'/dev/ttyUSB0','serial_baudrate':'460800','scan_topic':'/scan','base_frame':'base_link','laser_frame':'laser','publish_static_tf':'true','laser_x':'0.0','laser_y':'0.0','laser_z':'0.60','laser_roll':'0.0','laser_pitch':'0.0','laser_yaw':'0.0','node_output':'screen'}
    args=[DeclareLaunchArgument(k,default_value=v) for k,v in defaults.items()]
    lidar=Node(package='sllidar_ros2',executable='sllidar_node',name='sllidar_node',condition=IfCondition(L('start_lidar')),output=L('node_output'),remappings=[('scan',L('scan_topic'))],parameters=[{'channel_type':'serial','serial_port':L('serial_port'),'serial_baudrate':L('serial_baudrate'),'frame_id':L('laser_frame'),'inverted':False,'angle_compensate':True,'scan_mode':'Standard'}])
    # laser_z=0.60 m is the current approximate mounting height; replace it with the measured mounting height.
    tf=Node(package='tf2_ros',executable='static_transform_publisher',name='base_to_laser_tf',condition=IfCondition(L('publish_static_tf')),output=L('node_output'),arguments=['--x',L('laser_x'),'--y',L('laser_y'),'--z',L('laser_z'),'--roll',L('laser_roll'),'--pitch',L('laser_pitch'),'--yaw',L('laser_yaw'),'--frame-id',L('base_frame'),'--child-frame-id',L('laser_frame')])
    return LaunchDescription(args+[lidar,tf])
