import os
import tempfile

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    SetLaunchConfiguration,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
    GroupAction,
    LogInfo,
    Shutdown
)
from launch.conditions import IfCondition
from launch.event_handlers import OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression, PathJoinSubstitution

from launch_ros.actions import Node


def generate_launch_description():
    # Get the launch directory
    bringup_dir = get_package_share_directory('robot_navigo')
    launch_dir = os.path.join(bringup_dir, 'launch')

    # Declare the launch arguments
    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace', default_value='', description='Top-level namespace'
    )

    declare_use_namespace_cmd = DeclareLaunchArgument(
        'use_namespace',
        default_value='false',
        description='Whether to apply a namespace to the navigation stack',
    )

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true',
    )

    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map',
        default_value='none',
        description='Reference path to the map yaml file',
    )

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(bringup_dir, 'params', 'navigo_params.yaml'),
        description='Full path to the ROS2 parameters file to use for all launched nodes',
    )

    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='Automatically startup the nav2 stack',
    )

    declare_use_composition_cmd = DeclareLaunchArgument(
        'use_composition',
        default_value='True',
        description='Whether to use composed bringup',
    )

    declare_use_respawn_cmd = DeclareLaunchArgument(
        'use_respawn',
        default_value='False',
        description='Whether to respawn if a node crashes. Applied when composition is disabled.',
    )

    declare_log_level_cmd = DeclareLaunchArgument(
        'log_level', default_value='info', description='log level'
    )

    declare_robot_name_cmd = DeclareLaunchArgument(
        'robot_name', default_value='xg', description='name of the robot'
    )

    declare_platform_cmd = DeclareLaunchArgument(
        'platform',
        default_value='NO_SPECIFIED',
        description='platform of nav running on'
    )

    declare_mc_controller_type_cmd = DeclareLaunchArgument(
        'mc_controller_type',
        default_value='NO_SPECIFIED',
        description='mc controller type used in mc'
    )

    declare_communication_type_cmd = DeclareLaunchArgument(
        'communication_type',
        default_value='NO_SPECIFIED',
        description='type of communication to be used'
    )

    declare_tf_type_cmd = DeclareLaunchArgument(
        'tf_type',
        default_value='mujoco_tf',
        description='tf type used for odometry (mujoco_tf or localization_tf)'
    )

    set_map = SetLaunchConfiguration(
        'map',
        PythonExpression([
            "'",
            "",
            "' if '",
            LaunchConfiguration('map'),
            "' == 'none' else '",
            LaunchConfiguration('map'),
            "'"
        ])
    )

    set_use_sim_time = SetLaunchConfiguration(
        'use_sim_time',
        PythonExpression([
            "'",
            'true',
            "' if '",
            LaunchConfiguration('platform'),
            "' == 'GAZEBO' else '",
            LaunchConfiguration('use_sim_time'),
            "'"
        ])
    )

    set_params_file = SetLaunchConfiguration(
        'params_file',
        PythonExpression([
            "'",
            LaunchConfiguration('params_file'),
            "' if '",
            LaunchConfiguration('platform'),
            "' == 'GAZEBO' else '",
            LaunchConfiguration('params_file'),
            "'"
        ])
    )

    platform_log = LogInfo(
        msg=['platform: ', LaunchConfiguration('platform')]
    )

    map_log = LogInfo(
        msg=['map: ', LaunchConfiguration('map')]
    )

    use_sim_time_log = LogInfo(
        msg=['use_sim_time: ', LaunchConfiguration('use_sim_time')]
    )

    mc_controller_type_log = LogInfo(
        msg=['mc_controller_type: ', LaunchConfiguration('mc_controller_type')]
    )

    communication_type_log = LogInfo(
        msg=['communication_type: ', LaunchConfiguration('communication_type')]
    )

    navigo_params_log = LogInfo(
        msg=['navigo params file: ', LaunchConfiguration('params_file')]
    )

    platform_warn_log = LogInfo(
        condition=IfCondition(PythonExpression(["'", LaunchConfiguration('platform'), "' == 'NO_SPECIFIED'"])),
        msg="\033[1;31mWarning: You have not specified the 'platform'. [platform:=GAZEBO || platform:=UE platform:=NX_XG3588 || platform:=XG3588] Early Terminated\033[0m"
    )

    mc_controller_type_warn_log = LogInfo(
        condition=IfCondition(
            PythonExpression(["'", LaunchConfiguration('mc_controller_type'), "' == 'NO_SPECIFIED'"])),
        msg="\033[1;31mWarning: You have not specified the 'mc_controller_type'. [mc_controller_type:=RL_TRACK_VELOCITY || mc_controller_type:=RL_TRACK_PATH] Early Terminated\033[0m"
    )

    communication_type_warn_log = LogInfo(
        condition=IfCondition(
            PythonExpression(["'", LaunchConfiguration('communication_type'), "' == 'NO_SPECIFIED'"])),
        msg="\033[1;31mWarning: You have not specified the 'communication_type'. [communication_type:=UDP || communication_type:=LCM] Early Terminated\033[0m"
    )

    shutdown_if_no_platform_type = Shutdown(
        condition=IfCondition(PythonExpression(["'", LaunchConfiguration('platform'), "' == 'NO_SPECIFIED'"]))
    )

    shutdown_if_no_mc_controller_type = Shutdown(
        condition=IfCondition(PythonExpression(["'", LaunchConfiguration('mc_controller_type'), "' == 'NO_SPECIFIED'"]))
    )

    shutdown_if_no_communication_type = Shutdown(
        condition=IfCondition(PythonExpression(["'", LaunchConfiguration('communication_type'), "' == 'NO_SPECIFIED'"]))
    )

    # odom_communication_node = Node(
    #     package='robot_navigo',
    #     executable='odom_communication_node',
    #     output='screen'
    # )

    # custom_odom_baselink_node = Node(
    #     package='robot_navigo',
    #     executable='custom_odom_baselink_node',
    #     output='screen'
    # )

    # odom_tf_publisher_node = Node(
    #     package='robot_navigo',
    #     executable='odom_to_tf_broadcaster',
    #     output='screen'
    # )

    # cmd_cel_lcm_publisher_node = Node(
    #     package='robot_navigo',
    #     executable='vel_cmd_lcm_pub',
    #     output='screen'
    # )

    mode_status_pub_node = Node(
        package='robot_navigo',
        executable='mode_status_pub',
        output='screen'
    )

    load_vel_cmd_pub_node = GroupAction(
        actions=[
            Node(
                condition=IfCondition(PythonExpression(["'", LaunchConfiguration('communication_type'), "' == 'LCM'"])),
                package='robot_navigo',
                executable='vel_cmd_lcm_pub',
                parameters=[{'platform': LaunchConfiguration('platform')}],
                output='screen'
            ),
            Node(
                condition=IfCondition(PythonExpression([
                    "'", LaunchConfiguration('communication_type'), "' == 'UDP'",
                    " and ",
                    "'", LaunchConfiguration('mc_controller_type'), "' == 'RL_TRACK_VELOCITY'"
                ])),
                package='robot_navigo',
                executable='vel_cmd_udp_pub',
                parameters=[{'platform': LaunchConfiguration('platform')}],
                output='screen'
            ),
            # Node(
            #     condition=IfCondition(PythonExpression([
            #         "'", LaunchConfiguration('communication_type'), "' == 'UDP'",
            #         "and",
            #         "'", LaunchConfiguration('tf_type'), "' == 'localization_tf'",
            #         "and",
            #         "'", LaunchConfiguration('mc_controller_type'), "' == 'RL_TRACK_PATH'"
            #     ])),params_file
            #     package='robot_navigo',
            #     executable='vel_with_mc_trajectory_cmd_udp_pub',
            #     parameters=[{'platform': LaunchConfiguration('platform')}],
            #     output='screen'
            # )
        ]
    )

    # Create the launch configuration variables
    namespace = LaunchConfiguration('namespace')
    use_namespace = LaunchConfiguration('use_namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    map_yaml_file = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    autostart = LaunchConfiguration('autostart')
    use_composition = LaunchConfiguration('use_composition')
    use_respawn = LaunchConfiguration('use_respawn')

    bringup_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, 'bringup_launch.py')),
        launch_arguments={
            'namespace': namespace,
            'use_namespace': use_namespace,
            'map': map_yaml_file,
            'use_sim_time': use_sim_time,
            'params_file': params_file,
            'autostart': autostart,
            'use_composition': use_composition,
            'use_respawn': use_respawn,
        }.items(),
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_namespace_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_use_composition_cmd)
    ld.add_action(declare_use_respawn_cmd)
    ld.add_action(declare_log_level_cmd)
    ld.add_action(declare_robot_name_cmd)
    ld.add_action(declare_platform_cmd)
    ld.add_action(declare_mc_controller_type_cmd)
    ld.add_action(declare_communication_type_cmd)
    ld.add_action(declare_tf_type_cmd)

    ld.add_action(set_map)
    ld.add_action(set_use_sim_time)
    ld.add_action(set_params_file)

    ld.add_action(platform_log)
    ld.add_action(map_log)
    ld.add_action(mc_controller_type_log)
    ld.add_action(use_sim_time_log)
    ld.add_action(communication_type_log)
    ld.add_action(navigo_params_log)
    ld.add_action(platform_warn_log)
    ld.add_action(mc_controller_type_warn_log)
    ld.add_action(communication_type_warn_log)
    ld.add_action(shutdown_if_no_platform_type)
    ld.add_action(shutdown_if_no_mc_controller_type)
    ld.add_action(shutdown_if_no_communication_type)

    # ld.add_action(odom_communication_node)
    # ld.add_action(custom_odom_baselink_node)
    # ld.add_action(odom_tf_publisher_node)
    # ld.add_action(cmd_cel_lcm_publisher_node)
    ld.add_action(load_vel_cmd_pub_node)
    ld.add_action(mode_status_pub_node)

    # Add the actions to launch all of the navigation nodes
    ld.add_action(bringup_cmd)

    return ld
