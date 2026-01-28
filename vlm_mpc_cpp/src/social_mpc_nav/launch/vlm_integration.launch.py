from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Get package share directory
    pkg_share = FindPackageShare('social_mpc_nav')

    # Declare launch arguments
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=PathJoinSubstitution([
            pkg_share, 'config', 'vlm_integration.yaml'
        ]),
        description='Path to VLM integration configuration file'
    )

    # VLM Integration Node
    vlm_integration_node = Node(
        package='social_mpc_nav',
        executable='vlm_integration_node',
        name='vlm_integration_node',
        parameters=[LaunchConfiguration('config_file')],
        output='screen',
        emulate_tty=True
    )

    return LaunchDescription([
        config_file_arg,
        vlm_integration_node,
    ])
