import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # 获取配置文件路径
    localization_dir = get_package_share_directory('localization')
    config_file = os.path.join(localization_dir, 'config', 'config.yaml')
    
    return LaunchDescription([
        # 启动定位节点
        Node(
            package='localization',
            executable='localization_node',
            name='localization',
            output='screen',
            parameters=[config_file]
        ),

        # Node(
        #     package='localization',
        #     executable='localization_node',
        #     name='localization_map_server',
        #     output='screen',
        #     parameters=[config_file]
        # )
        
        # 静态TF发布器
        Node(
            name='lidar_tf',
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0.0', '0.0', '0.0', '0', '0', '0', '1', 'odom', 'livox_frame']
        )
    ]) 