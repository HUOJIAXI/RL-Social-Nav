import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Get package directory
    pkg_dir = get_package_share_directory('dwa_local_planner')

    # Default config file
    default_config_file = os.path.join(pkg_dir, 'config', 'dwa_controller.yaml')

    # Declare launch arguments
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=default_config_file,
        description='Path to DWA controller config file'
    )

    goal_x_arg = DeclareLaunchArgument(
        'goal_x',
        default_value='10.0',
        description='Goal x coordinate'
    )

    goal_y_arg = DeclareLaunchArgument(
        'goal_y',
        default_value='5.0',
        description='Goal y coordinate'
    )

    # DWA Controller Node
    dwa_controller_node = Node(
        package='dwa_local_planner',
        executable='dwa_controller_node',
        name='dwa_controller_node',
        output='screen',
        parameters=[
            LaunchConfiguration('config_file'),
            {
                'goal_x': LaunchConfiguration('goal_x'),
                'goal_y': LaunchConfiguration('goal_y'),
            }
        ],
        remappings=[
            ('/cmd_vel', '/task_generator_node/tiago_base/cmd_vel'),
        ]
    )

    return LaunchDescription([
        config_file_arg,
        goal_x_arg,
        goal_y_arg,
        dwa_controller_node,
    ])
