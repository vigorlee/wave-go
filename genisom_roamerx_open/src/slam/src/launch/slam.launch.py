import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess


def generate_launch_description():
    parameter_config = os.path.join(get_package_share_directory(
        'robot_slam'), 'config', 'config.yaml')
    default_rviz_config_path = os.path.join(
        get_package_share_directory('robot_slam'), 'rviz', 'mapping.rviz')
    print(parameter_config)
    return LaunchDescription([
        Node(
            package='robot_slam',
            executable='mapping',
            parameters=[
                parameter_config
            ]
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', default_rviz_config_path]
        )
    ])
