"""Online mapping/navigation; deliberately no map_server or ground-truth TF."""
import os
from pathlib import Path
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node


def generate_launch_description():
    root=Path(__file__).resolve().parents[1]
    params=str(root/'config/go2w_exploration_nav.yaml')
    output=os.environ.get('GO2W_EXPLORATION_OUTPUT',str(root/'artifacts/Wavegov2_online_exploration'))
    nodes=[Node(package='robot_slam',executable='mapping',name='laser_mapping',
                parameters=[str(root/'config/go2w_online_slam.yaml')],output='screen',
                remappings=[('/tf','/wavego/slam_tf'),('/world_points','/wavego/slam/registered_points'),('/slam_odom','/wavego/slam/odom')]),
           ExecuteProcess(cmd=['/usr/bin/python3',str(root/'scripts/go2w_online_exploration.py'),'--output',output],output='screen'),
           ExecuteProcess(cmd=['bash','-c',"sleep 2; ros2 service call /slam_state_service robots_dog_msgs/srv/MapState '{data: 3}'"],output='screen')]
    servers=[('navigo_path_controller','controller_server'),('navigo_path_planner','planner_server'),
             ('navigo_behaviors','behavior_server'),('navigo_velocity_optimizer','velocity_optimizer'),
             ('navigo_bt_navigator','bt_navigator')]
    for package,name in servers:
        remap=[]
        if name in ('controller_server','behavior_server'):remap=[('cmd_vel','cmd_vel_nav')]
        if name=='velocity_optimizer':remap=[('cmd_vel','cmd_vel_nav'),('cmd_vel_smoothed','cmd_vel')]
        executable='velocity_smoother' if name=='velocity_optimizer' else name
        nodes.append(Node(package=package,executable=executable,name=name,parameters=[params],remappings=remap,output='screen'))
    nodes.extend([Node(package='nav2_lifecycle_manager',executable='lifecycle_manager',name='lifecycle_manager_navigation',
                       parameters=[{'autostart':True,'node_names':[x[1] for x in servers],'bond_timeout':4.0}],output='screen'),
                  Node(package='robot_navigo',executable='vel_cmd_lcm_pub',parameters=[{'platform':'UE'}],output='screen'),
                  Node(package='rviz2',executable='rviz2',arguments=['-d',str(root/'config/go2w_online_mapping.rviz')],output='screen')])
    return LaunchDescription(nodes)
