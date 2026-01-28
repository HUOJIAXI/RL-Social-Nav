import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Get package directories
    dwa_pkg_dir = get_package_share_directory('dwa_local_planner')
    social_nav_pkg_dir = get_package_share_directory('social_mpc_nav')

    # Config files
    dwa_config = os.path.join(dwa_pkg_dir, 'config', 'dwa_controller.yaml')
    gt_loc_config = os.path.join(social_nav_pkg_dir, 'config', 'gt_localization.yaml')
    global_planner_config = os.path.join(social_nav_pkg_dir, 'config', 'global_planner.yaml')

    # Get workspace root directory (parent of dwa_local_planner package)
    workspace_root = os.path.dirname(os.path.dirname(os.path.dirname(dwa_pkg_dir)))
    rviz_config = os.path.join(workspace_root, 'rviz_config', 'mpc.rviz')

    # Declare launch arguments
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

    log_directory_arg = DeclareLaunchArgument(
        'log_directory',
        default_value='~/ros2_logs/social_mpc_nav',
        description='Directory for CSV log files'
    )

    # Ground Truth Localization Node
    gt_localization_node = Node(
        package='social_mpc_nav',
        executable='gt_localization_node',
        name='gt_localization_node',
        output='screen',
        parameters=[gt_loc_config]
    )

    # Global Planner Node
    global_planner_node = Node(
        package='social_mpc_nav',
        executable='global_planner_node',
        name='global_planner_node',
        output='screen',
        parameters=[
            global_planner_config,
            {
                'goal_x': LaunchConfiguration('goal_x'),
                'goal_y': LaunchConfiguration('goal_y'),
            }
        ]
    )

    # DWA Controller Node
    dwa_controller_node = Node(
        package='dwa_local_planner',
        executable='dwa_controller_node',
        name='dwa_controller_node',
        output='screen',
        parameters=[
            dwa_config,
            {
                'goal_x': LaunchConfiguration('goal_x'),
                'goal_y': LaunchConfiguration('goal_y'),
                'map_frame': 'map',
                'base_frame': 'tiago_base/base_footprint',
            }
        ]
    )

    # Pedestrian Logger Node
    pedestrian_logger_node = Node(
        package='social_mpc_nav',
        executable='pedestrian_logger_node',
        name='pedestrian_logger_node',
        output='screen',
        parameters=[
            {
                'log_directory': LaunchConfiguration('log_directory'),
                'human_states_topic': '/task_generator_node/human_states',
                'log_rate_hz': 10.0,
                'enable_logging': True,
            }
        ]
    )

    # RViz Node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen'
    )

    return LaunchDescription([
        goal_x_arg,
        goal_y_arg,
        log_directory_arg,
        gt_localization_node,
        global_planner_node,
        dwa_controller_node,
        pedestrian_logger_node,
        rviz_node,
    ])
