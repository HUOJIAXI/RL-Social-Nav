#!/usr/bin/env python3
"""
Social MPC Navigation Launch File for Jackal Robot

This launch file starts the MPC navigation system for Jackal robot:
1. global_planner_node - Plans waypoint-based path to goal
2. mpc_controller_node - Tracks waypoints with obstacle/pedestrian avoidance
   (subscribes directly to /person_tracker/person_info for people detection)

Note: This launch file assumes:
- Jackal simulation is already running
- Odometry is published at /task_generator_node/jackal/odom
- LaserScan is published at /task_generator_node/jackal/lidar
- People tracking is published at /task_generator_node/people
- TF transforms (map -> jackal/odom -> jackal/base_link) are already published

Usage:
    # Basic usage with default goal
    ros2 launch social_mpc_nav mpc_jackal.launch.py

    # With custom goal
    ros2 launch social_mpc_nav mpc_jackal.launch.py \
        goal_x:=5.0 goal_y:=3.0

    # With debug logging and custom MPC parameters
    ros2 launch social_mpc_nav mpc_jackal.launch.py \
        goal_x:=5.0 goal_y:=3.0 \
        enable_debug_logging:=true \
        num_rollouts:=100 \
        N:=20
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Get the package directory
    pkg_dir = get_package_share_directory('social_mpc_nav')

    # Default config file path for Jackal
    default_jackal_config = os.path.join(
        pkg_dir, 'config', 'navigation_params_jackal.yaml')

    # Declare launch arguments
    goal_x_arg = DeclareLaunchArgument(
        'goal_x',
        default_value='10.0',
        description='Goal X coordinate in map frame (meters)'
    )

    goal_y_arg = DeclareLaunchArgument(
        'goal_y',
        default_value='5.0',
        description='Goal Y coordinate in map frame (meters)'
    )

    goal_tolerance_arg = DeclareLaunchArgument(
        'goal_tolerance',
        default_value='0.2',
        description='Distance threshold to consider goal reached (meters)'
    )

    waypoint_spacing_arg = DeclareLaunchArgument(
        'waypoint_spacing',
        default_value='1.0',
        description='Distance between waypoints in global path (meters)'
    )

    log_directory_arg = DeclareLaunchArgument(
        'log_directory',
        default_value='~/ros2_logs/social_mpc_nav_jackal',
        description='Directory for CSV log files'
    )

    enable_debug_logging_arg = DeclareLaunchArgument(
        'enable_debug_logging',
        default_value='false',
        description='Enable detailed MPC debug logging'
    )

    # MPC tuning arguments
    num_rollouts_arg = DeclareLaunchArgument(
        'num_rollouts',
        default_value='80',
        description='Number of MPC rollouts (increase for crowds or complex scenarios)'
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

    default_v_max_arg = DeclareLaunchArgument(
        'default_v_max',
        default_value='0.8',
        description='Maximum linear velocity (m/s)'
    )

    omega_max_arg = DeclareLaunchArgument(
        'omega_max',
        default_value='1.5',
        description='Maximum angular velocity (rad/s)'
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
    default_v_max = LaunchConfiguration('default_v_max')
    omega_max = LaunchConfiguration('omega_max')

    # Info message
    launch_info = LogInfo(
        msg=[
            '\n',
            '==================================================\n',
            'Social MPC Navigation for Jackal Robot\n',
            '==================================================\n',
            'Goal: (', goal_x, ', ', goal_y, ') tolerance: ', goal_tolerance, ' m\n',
            'Waypoint spacing: ', waypoint_spacing, ' m\n',
            'Max velocities: v=', default_v_max, ' m/s, omega=', omega_max, ' rad/s\n',
            'MPC params: N=', N, ', rollouts=', num_rollouts, '\n',
            'Debug logging: ', enable_debug_logging, '\n',
            'Log directory: ', log_directory, '\n',
            '==================================================\n'
        ]
    )

    # Node 1: Global Planner
    # Plans waypoint-based path from current position to goal
    global_planner_node = Node(
        package='social_mpc_nav',
        executable='global_planner_node',
        name='global_planner_node',
        output='screen',
        parameters=[
            default_jackal_config,
            {
                'goal_x': goal_x,
                'goal_y': goal_y,
                'waypoint_spacing': waypoint_spacing,
                'log_directory': log_directory,
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
    #         default_jackal_config,
    #         {
    #             'log_directory': log_directory,
    #         }
    #     ],
    #     prefix=['stdbuf -o L'],
    # )

    # Node 3: MPC Controller
    # Tracks waypoints from global path while avoiding obstacles and pedestrians
    # Uses social contract system for adaptive behavior
    mpc_controller_node = Node(
        package='social_mpc_nav',
        executable='mpc_controller_node',
        name='mpc_controller_node',
        output='screen',
        parameters=[
            default_jackal_config,
            {
                'goal_x': goal_x,
                'goal_y': goal_y,
                'goal_tolerance': goal_tolerance,
                'log_directory': log_directory,
                'enable_debug_logging': enable_debug_logging,
                'num_rollouts': num_rollouts,
                'N': N,
                'w_obstacle': w_obstacle,
                'default_v_max': default_v_max,
                'omega_max': omega_max,
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
        default_v_max_arg,
        omega_max_arg,

        # Info message
        launch_info,

        # Nodes
        global_planner_node,
        # people_adapter_node,  # Removed - MPC now subscribes to /person_tracker/person_info directly
        mpc_controller_node,
    ])
