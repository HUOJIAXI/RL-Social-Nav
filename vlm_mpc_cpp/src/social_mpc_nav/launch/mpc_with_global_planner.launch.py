#!/usr/bin/env python3
"""
Complete launch file for Social MPC Navigation with Global Planner.

This launch file starts:
1. global_planner_node - Plans waypoint-based path to goal
2. mpc_controller_node - Tracks waypoints with obstacle/pedestrian avoidance
   (subscribes directly to /person_tracker/person_info for people detection)

Usage:
    # Basic usage
    ros2 launch social_mpc_nav mpc_with_global_planner.launch.py

    # With custom goal
    ros2 launch social_mpc_nav mpc_with_global_planner.launch.py \
        goal_x:=-5.0 goal_y:=-15.0

    # With debug logging
    ros2 launch social_mpc_nav mpc_with_global_planner.launch.py \
        goal_x:=-5.0 goal_y:=-15.0 enable_debug_logging:=true
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    # Get the package directory
    pkg_dir = get_package_share_directory('social_mpc_nav')

    # Default config file paths
    default_global_planner_config = os.path.join(
        pkg_dir, 'config', 'global_planner.yaml')
    # default_people_adapter_config = os.path.join(
    #     pkg_dir, 'config', 'people_adapter.yaml')  # No longer needed
    default_mpc_config = os.path.join(
        pkg_dir, 'config', 'mpc_controller.yaml')

    # Declare launch arguments
    goal_x_arg = DeclareLaunchArgument(
        'goal_x',
        default_value='-5.0',
        description='Goal X coordinate (meters)'
    )

    goal_y_arg = DeclareLaunchArgument(
        'goal_y',
        default_value='-15.0',
        description='Goal Y coordinate (meters)'
    )

    goal_tolerance_arg = DeclareLaunchArgument(
        'goal_tolerance',
        default_value='0.15',
        description='Distance threshold to consider goal reached (meters)'
    )

    waypoint_spacing_arg = DeclareLaunchArgument(
        'waypoint_spacing',
        default_value='1.0',
        description='Distance between waypoints in global path (meters)'
    )

    log_directory_arg = DeclareLaunchArgument(
        'log_directory',
        default_value='~/ros2_logs/social_mpc_nav',
        description='Directory for CSV log files'
    )

    enable_debug_logging_arg = DeclareLaunchArgument(
        'enable_debug_logging',
        default_value='false',
        description='Enable detailed MPC debug logging'
    )

    # MPC tuning arguments for crowded scenarios
    num_rollouts_arg = DeclareLaunchArgument(
        'num_rollouts',
        default_value='100',
        description='Number of MPC rollouts (increase for crowds)'
    )

    N_arg = DeclareLaunchArgument(
        'N',
        default_value='15',
        description='MPC horizon steps'
    )

    w_obstacle_arg = DeclareLaunchArgument(
        'w_obstacle',
        default_value='3.0',
        description='Obstacle avoidance weight'
    )

    # Launch configurations
    goal_x = LaunchConfiguration('goal_x')
    goal_y = LaunchConfiguration('goal_y')
    goal_tolerance = LaunchConfiguration('goal_tolerance')
    waypoint_spacing = LaunchConfiguration('waypoint_spacing')
    log_directory = LaunchConfiguration('log_directory')
    enable_debug_logging = LaunchConfiguration('enable_debug_logging')
    num_rollouts = LaunchConfiguration('num_rollouts')
    N = LaunchConfiguration('N')
    w_obstacle = LaunchConfiguration('w_obstacle')

    # Info message
    launch_info = LogInfo(
        msg=[
            '\n',
            '==================================================\n',
            'Social MPC with Global Planner\n',
            '==================================================\n',
            'Goal: (', goal_x, ', ', goal_y, ') tolerance: ', goal_tolerance, ' m\n',
            'Waypoint spacing: ', waypoint_spacing, ' m\n',
            'Debug logging: ', enable_debug_logging, '\n',
            'Log directory: ', log_directory, '\n',
            '==================================================\n'
        ]
    )

    # Node 1: Global Planner
    # Plans waypoint-based path from start to goal
    global_planner_node = Node(
        package='social_mpc_nav',
        executable='global_planner_node',
        name='global_planner_node',
        output='screen',
        parameters=[
            default_global_planner_config,
            {
                'goal_x': goal_x,
                'goal_y': goal_y,
                'waypoint_spacing': waypoint_spacing,
                'odom_topic': '/task_generator_node/tiago_base/odom',
                'map_frame': 'map',
                'odom_frame': 'tiago_base/odom',
            }
        ],
        prefix=['stdbuf -o L'],
    )

    # Node 2: People Adapter - REMOVED
    # The people adapter is no longer needed as MPC controllers now subscribe
    # directly to /person_tracker/person_info from the 3D detector
    # people_adapter_node = Node(
    #     package='social_mpc_nav',
    #     executable='people_adapter_node',
    #     name='people_adapter_node',
    #     output='screen',
    #     parameters=[
    #         default_people_adapter_config,
    #         {
    #             'log_directory': log_directory,
    #         }
    #     ],
    #     prefix=['stdbuf -o L'],
    # )

    # Node 3: PointCloud to LaserScan Converter (NOT NEEDED for tiago_base)
    # tiago_base already has a native LaserScan at /task_generator_node/tiago_base/lidar
    # Keeping this commented out in case needed for other robots
    # pointcloud_to_laserscan_node = Node(
    #     package='pointcloud_to_laserscan',
    #     executable='pointcloud_to_laserscan_node',
    #     name='pointcloud_to_laserscan',
    #     remappings=[
    #         ('cloud_in', '/task_generator_node/tiago_base/lidar/points'),
    #         ('scan', '/scan_2d'),
    #     ],
    #     parameters=[{
    #         'target_frame': 'tiago_base/base_footprint',
    #         'transform_tolerance': 0.01,
    #         'min_height': 0.1,
    #         'max_height': 0.5,
    #         'angle_min': -3.14159,
    #         'angle_max': 3.14159,
    #         'angle_increment': 0.00872,
    #         'scan_time': 0.1,
    #         'range_min': 0.25,
    #         'range_max': 30.0,
    #         'use_inf': True,
    #     }],
    # )

    # Node 4: Ground-Truth Localization
    # Extracts pedestrian (actor) positions from Gazebo ModelStates
    # Publishes map -> odom TF transform (identity)
    # Robot odometry comes from task_generator_node (not from ModelStates)
    gt_localization_node = Node(
        package='social_mpc_nav',
        executable='gt_localization_node',
        name='gt_localization_node',
        output='screen',
        parameters=[
            {
                'robot_model_name': 'tiago_base',
                'map_frame': 'map',
                'odom_frame': 'tiago_base/odom',
                'base_link_frame': 'tiago_base/base_footprint',
                'world_frame': 'world',
                'model_states_topic': '/gazebo/model_states'
            }
        ],
        prefix=['stdbuf -o L'],
    )

    # Node 5: MPC Controller
    # Tracks waypoints from global path while avoiding obstacles/pedestrians
    mpc_controller_node = Node(
        package='social_mpc_nav',
        executable='mpc_controller_node',
        name='mpc_controller_node',
        output='screen',
        parameters=[
            default_mpc_config,
            {
                'goal_x': goal_x,
                'goal_y': goal_y,
                'goal_tolerance': goal_tolerance,
                'log_directory': log_directory,
                'enable_debug_logging': enable_debug_logging,
                'num_rollouts': num_rollouts,
                'N': N,
                'w_obstacle': w_obstacle,
                'cmd_vel_topic': '/task_generator_node/tiago_base/cmd_vel',
                'odom_topic': '/task_generator_node/tiago_base/odom',
                'scan_topic': '/task_generator_node/tiago_base/lidar',
                'min_valid_laser_range': 0.5,
                'map_frame': 'map',
                'odom_frame': 'tiago_base/odom',
            }
        ],
        prefix=['stdbuf -o L'],
    )

    return LaunchDescription([
        # Launch arguments
        goal_x_arg,
        goal_y_arg,
        goal_tolerance_arg,
        waypoint_spacing_arg,
        log_directory_arg,
        enable_debug_logging_arg,
        num_rollouts_arg,
        N_arg,
        w_obstacle_arg,

        # Info message
        launch_info,

        # Nodes
        global_planner_node,
        # people_adapter_node,  # Removed - MPC now subscribes to /person_tracker/person_info directly
        # pointcloud_to_laserscan_node,  # Not needed for tiago_base
        gt_localization_node,
        mpc_controller_node,
    ])
