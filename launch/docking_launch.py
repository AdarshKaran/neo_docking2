# Neobotix GmbH

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    params_file = LaunchConfiguration(
        'params_file', 
        default=os.path.join(
            get_package_share_directory('neo_docking2'), 
            'config', 
            'docking.yaml'
        )
    )
    dock_database_file = LaunchConfiguration(
        'dock_database_file', 
        default=os.path.join(
            get_package_share_directory('neo_docking2'), 
            'config', 
            'dock_database.yaml'
        )
    )
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )
    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(get_package_share_directory('neo_docking2'), 'config', 'docking.yaml'),
        description='Full path to the docking parameters file to use'
    )
    declare_dock_database_cmd = DeclareLaunchArgument(
        'dock_database_file',
        default_value=os.path.join(get_package_share_directory('neo_docking2'), 'config', 'dock_database.yaml'),
        description='Full path to the docking database file to use'
    )

    # RewrittenYaml to substitute the dock_database parameter in the YAML file
    rewritten_params = RewrittenYaml(
        source_file=params_file,
        root_key='',
        param_rewrites={'dock_database': dock_database_file},
        convert_types=True
    )

    # Path to the scripts
    scripts_path = os.path.join(get_package_prefix('neo_docking2'), 'lib', 'neo_docking2')

    return LaunchDescription([
        declare_use_sim_time_cmd,
        declare_params_file_cmd,
        declare_dock_database_cmd,

        Node(
            package='opennav_docking',
            executable='opennav_docking',
            name='docking_server',
            output='screen',
            parameters=[rewritten_params],
            remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')]
        ),

        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_docking',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'autostart': True,
                'node_names': ['docking_server']
            }]
        ),
        # Publish pose script
        Node(
            package='neo_docking2',
            executable=os.path.join(scripts_path, 'publish_pose.py'),
            name='pose_publisher',
            output='screen'
        ),
        # Save pose to YAML service script
        Node(
            package='neo_docking2',
            executable=os.path.join(scripts_path, 'save_pose_to_yaml.py'),
            name='save_pose_to_yaml',
            output='screen'
        ),
    ])

