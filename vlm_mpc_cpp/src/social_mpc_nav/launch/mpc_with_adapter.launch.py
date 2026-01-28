#!/usr/bin/env python3
"""
Complete launch file for Social MPC Navigation with People Adapter.

DEPRECATED: This launch file is no longer needed. The people adapter has been
removed as MPC controllers now subscribe directly to /person_tracker/person_info.
Use one of the other launch files instead (mpc_with_global_planner.launch.py, etc.)

This launch file previously started:
1. people_adapter_node - Converts people_msgs to People2D format (REMOVED)
2. mpc_controller_node - Social MPC controller with obstacle avoidance

Usage:
    ros2 launch social_mpc_nav mpc_with_adapter.launch.py

    # With custom goal:
    ros2 launch social_mpc_nav mpc_with_adapter.launch.py goal_x:=15.0 goal_y:=10.0

    # With custom config files:
    ros2 launch social_mpc_nav mpc_with_adapter.launch.py \
        people_adapter_config:=/path/to/people_adapter.yaml \
        mpc_config:=/path/to/mpc_controller.yaml

    # Change log directory:
    ros2 launch social_mpc_nav mpc_with_adapter.launch.py \
        log_directory:=/tmp/my_logs
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    # Get the package directory
    pkg_dir = get_package_share_directory('social_mpc_nav')

    # Default config file paths
    default_people_adapter_config = os.path.join(
        pkg_dir, 'config', 'people_adapter.yaml')
    default_mpc_config = os.path.join(
        pkg_dir, 'config', 'mpc_controller.yaml')

    # Declare launch arguments
    people_adapter_config_arg = DeclareLaunchArgument(
        'people_adapter_config',
        default_value=default_people_adapter_config,
        description='Path to people_adapter configuration file'
    )

    mpc_config_arg = DeclareLaunchArgument(
        'mpc_config',
        default_value=default_mpc_config,
        description='Path to mpc_controller configuration file'
    )

    goal_x_arg = DeclareLaunchArgument(
        'goal_x',
        default_value='-5.0',
        description='Goal X coordinate (meters)'
    )

    goal_y_arg = DeclareLaunchArgument(
        'goal_y',
        default_value='5.0',
        description='Goal Y coordinate (meters)'
    )

    goal_tolerance_arg = DeclareLaunchArgument(
        'goal_tolerance',
        default_value='0.15',
        description='Distance threshold to consider goal reached (meters)'
    )

    log_directory_arg = DeclareLaunchArgument(
        'log_directory',
        default_value='~/ros2_logs/social_mpc_nav',
        description='Directory for CSV log files'
    )

    input_people_topic_arg = DeclareLaunchArgument(
        'input_people_topic',
        default_value='/task_generator_node/people',
        description='Input topic with people_msgs/msg/People'
    )

    output_people_topic_arg = DeclareLaunchArgument(
        'output_people_topic',
        default_value='/crowd/people2d',
        description='Output topic with social_mpc_nav/msg/People2D'
    )

    # Launch configurations
    people_adapter_config = LaunchConfiguration('people_adapter_config')
    mpc_config = LaunchConfiguration('mpc_config')
    goal_x = LaunchConfiguration('goal_x')
    goal_y = LaunchConfiguration('goal_y')
    goal_tolerance = LaunchConfiguration('goal_tolerance')
    log_directory = LaunchConfiguration('log_directory')
    input_people_topic = LaunchConfiguration('input_people_topic')
    output_people_topic = LaunchConfiguration('output_people_topic')

    # Info message
    launch_info = LogInfo(
        msg=[
            '\n',
            '==================================================\n',
            'Social MPC Navigation with People Adapter\n',
            '==================================================\n',
            'Goal: (', goal_x, ', ', goal_y, ') tolerance: ', goal_tolerance, ' m\n',
            'People input: ', input_people_topic, '\n',
            'People output: ', output_people_topic, '\n',
            'Log directory: ', log_directory, '\n',
            '==================================================\n'
        ]
    )

    # Node 1: People Adapter
    # Converts people_msgs/msg/People to social_mpc_nav/msg/People2D
    people_adapter_node = Node(
        package='social_mpc_nav',
        executable='people_adapter_node',
        name='people_adapter_node',
        output='screen',
        parameters=[
            people_adapter_config,
            {
                'input_topic': input_people_topic,
                'output_topic': output_people_topic,
                'log_directory': log_directory,
            }
        ],
        # Add prefix for colored output (optional)
        prefix=['stdbuf -o L'],  # Line buffered output for better logging
    )

    # Node 2: MPC Controller
    # Social MPC with obstacle avoidance
    mpc_controller_node = Node(
        package='social_mpc_nav',
        executable='mpc_controller_node',
        name='mpc_controller_node',
        output='screen',
        parameters=[
            mpc_config,
            {
                'goal_x': goal_x,
                'goal_y': goal_y,
                'goal_tolerance': goal_tolerance,
                'log_directory': log_directory,
                'crowd_topic': output_people_topic,  # Must match adapter output
            }
        ],
        # Add prefix for colored output (optional)
        prefix=['stdbuf -o L'],  # Line buffered output for better logging
    )

    return LaunchDescription([
        # Launch arguments
        people_adapter_config_arg,
        mpc_config_arg,
        goal_x_arg,
        goal_y_arg,
        goal_tolerance_arg,
        log_directory_arg,
        input_people_topic_arg,
        output_people_topic_arg,

        # Info message
        launch_info,

        # Nodes
        people_adapter_node,
        mpc_controller_node,
    ])
