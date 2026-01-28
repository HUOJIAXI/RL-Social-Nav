import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Get the package directory
    pkg_dir = get_package_share_directory('social_mpc_nav')

    # Declare launch arguments for config file paths
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=os.path.join(pkg_dir, 'config', 'navigation_params.yaml'),
        description='Path to the configuration file'
    )

    # Optional: Allow overriding specific parameters
    goal_x_arg = DeclareLaunchArgument("goal_x", default_value="")
    goal_y_arg = DeclareLaunchArgument("goal_y", default_value="")

    # Load configuration file
    config_file = LaunchConfiguration('config_file')

    gt_localization = Node(
        package="social_mpc_nav",
        executable="gt_localization_node",
        name="gt_localization_node",
        output="screen",
        parameters=[config_file]
    )

    crowd_state = Node(
        package="social_mpc_nav",
        executable="crowd_state_node",
        name="crowd_state_node",
        output="screen",
        parameters=[config_file]
    )

    # MPC controller with config file and optional parameter overrides
    mpc_params = [config_file]

    # Add parameter overrides if provided
    override_params = {}
    goal_x_config = LaunchConfiguration("goal_x")
    goal_y_config = LaunchConfiguration("goal_y")

    mpc_controller = Node(
        package="social_mpc_nav",
        executable="mpc_controller_node",
        name="mpc_controller_node",
        output="screen",
        parameters=[
            config_file,
            {
                'goal_x': goal_x_config,
                'goal_y': goal_y_config,
            }
        ]
    )

    return LaunchDescription([
        config_file_arg,
        goal_x_arg,
        goal_y_arg,
        gt_localization,
        crowd_state,
        mpc_controller
    ])

