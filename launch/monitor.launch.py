from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os

def generate_launch_description():
    output_arg = DeclareLaunchArgument(
        'output_path',
        default_value='recorded_data'
    )
    planning_shield_enabled = DeclareLaunchArgument(
        'planning_shield_enabled',
        default_value='false',
    )
    perception_shield_enabled = DeclareLaunchArgument(
        'perception_shield_enabled',
        default_value='false',
    )
    no_sim = DeclareLaunchArgument(
        'no_sim',
        default_value='1',
    )
    config = os.path.join(
        os.getenv('COLCON_PREFIX_PATH').split(':')[0],
        '..', 'config', 'default.yaml'
    )

    return LaunchDescription([
        output_arg,
        planning_shield_enabled,
        perception_shield_enabled,
        no_sim,
        Node(
            package='aw_runtime_monitor',
            executable='monitor',
            name='aw_recorder',
            output='screen',
            parameters=[config,  # <-- default config file
                        {'output_path': LaunchConfiguration('output_path'),
                         'planning_shield_enabled': LaunchConfiguration('planning_shield_enabled'),
                         'perception_shield_enabled': LaunchConfiguration('perception_shield_enabled'),
                         'no_sim': LaunchConfiguration('no_sim')}
                        ]  
        )
    ])
