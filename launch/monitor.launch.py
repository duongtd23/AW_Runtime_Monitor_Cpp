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
    # planning_shield = DeclareLaunchArgument(
    #     'planning_shield',
    #     default_value='false',
    # )
    # perception_shield = DeclareLaunchArgument(
    #     'perception_shield',
    #     default_value='false',
    # )
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
        # planning_shield,
        # perception_shield,
        no_sim,
        Node(
            package='aw_runtime_monitor',
            executable='aw_rt_monitor',
            name='aw_recorder',
            output='screen',
            parameters=[config,  # <-- default config file
                        {'output_path': LaunchConfiguration('output_path'),
                        #  'planning_shield': LaunchConfiguration('planning_shield'),
                        #  'perception_shield': LaunchConfiguration('perception_shield'),
                         'no_sim': LaunchConfiguration('no_sim')}
                        ]  
        )
    ])
