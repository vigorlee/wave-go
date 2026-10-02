"""Independent measured LIO map alongside the validated terrain controller."""
import os
from pathlib import Path
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():
    root=Path(__file__).resolve().parents[2]
    return LaunchDescription([
        Node(package='robot_slam',executable='mapping',name='laser_mapping',parameters=[str(root/'config/office_lio_dense.yaml')],remappings=[('/tf','/wavego/slam_tf'),('/world_points','/wavego/slam/registered_points'),('/map_points','/wavego/slam/ikd_map'),('/slam_odom','/wavego/slam/odom')],output='screen'),
        ExecuteProcess(cmd=['/usr/bin/python3',str(root/'scripts/go2w_online_exploration.py'),'--output',os.environ['GO2W_MAPPING_OUTPUT'],'--ros-args','-r','/livox/lidar:=/wavego/slam/raw_lidar','-r','/map:=/wavego/mapping/grid'],additional_env={'GO2W_MAPPING_ONLY':'1','GO2W_COSMOS_FARTHEST':'0','GO2W_MAPPING_FRAME':'lio_map','GO2W_MAPPING_BASE_FRAME':'lio_base_link'},output='screen'),
        ExecuteProcess(cmd=['bash','-c',"sleep 2; ros2 service call /slam_state_service robots_dog_msgs/srv/MapState '{data: 3}'"],output='screen')])
